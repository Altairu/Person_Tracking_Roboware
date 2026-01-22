/****************************************************************************
 * バイアス推定付きカルマンフィルタ (Augmented State Kalman Filter)
 * - 修正版: calibration関数を追加
 * - 状態ベクトル: x = [位置, 速度, 加速度バイアス]
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <MadgwickAHRS.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#define TS_FREQ (19200000)
#define SAMPLERATE (1920)
#define ARANGE (16)
#define GRANGE (1000)

/* KFパラメータチューニング */
#define VAR_ACC_NOISE (0.001634f)
#define VAR_BIAS_NOISE (0.00000001f) /* Approx. GyroVar/100 */
#define VAR_POS_MEAS (0.00000001f)   /* Keep manual */
#define VAR_VEL_MEAS (0.001f)        /* Keep manual */

/* 座標系反転オプション */
#define INVERT_PC_AXIS (0)

#define GH(a) (gyro[(a)] / 2.f)

/* PCから送られてくる補正データ構造体 */
struct pc_correction_s {
  uint8_t header[2];    /* 0xAA, 0x55 */
  float vel_now[3];     /* PCが計算した現在の速度 */
  float pos_delayed[3]; /* PCが計算した「過去(10秒前)」の位置 */
  uint8_t checksum;     /* チェックサム */
} __attribute__((packed));

/* 履歴保存用の構造体 */
#define HISTORY_SEC 12
#define HISTORY_FREQ 10
#define HISTORY_LEN (HISTORY_SEC * HISTORY_FREQ)

struct position_history_s {
  float pos[3];
  uint32_t timestamp;
};

/* 状態ベクトル拡張: [0]Pos, [1]Vel, [2]AccelBias */
struct kf_state_s {
  float x[3];
  float P[3][3];
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pospredict_s {
  uint32_t last_ts;
  float posquat[4];
  float gravity;
  float gbias[3];
  float calibtime;
  struct kf_state_s kf[3]; /* 3軸独立 */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pospredict_s g_inst;
static struct position_history_s g_history[HISTORY_LEN];
static int g_hist_idx = 0;

/****************************************************************************
 * KF Logic Functions
 ****************************************************************************/

/* KF初期化 */
void kf_init(struct kf_state_s *kf) {
  memset(kf, 0, sizeof(struct kf_state_s));
  /* 初期共分散 */
  kf->P[0][0] = 1.0f;
  kf->P[1][1] = 1.0f;
  kf->P[2][2] = 0.1f; /* バイアスの初期不確かさ */
}

/* 予測ステップ (Time Update) */
void kf_predict(struct kf_state_s *kf, float raw_acc, float dt) {
  float p = kf->x[0];
  float v = kf->x[1];
  float b = kf->x[2];

  /* 入力加速度(バイアス除去前)から推定バイアスを引く */
  float effective_acc = raw_acc - b;

  /* 1. 状態予測 */
  kf->x[0] = p + v * dt + 0.5f * effective_acc * dt * dt;
  kf->x[1] = v + effective_acc * dt;
  kf->x[2] = b; /* バイアスは一定と仮定（ノイズで変動） */

  /* 2. 共分散予測 P = F P F^T + Q */
  float dt2 = dt * dt;
  float F00 = 1, F01 = dt, F02 = -0.5f * dt2;
  float F10 = 0, F11 = 1, F12 = -dt;
  float F20 = 0, F21 = 0, F22 = 1;

  /* Q行列（プロセスノイズ） */
  float q_acc = VAR_ACC_NOISE;
  float q_bias = VAR_BIAS_NOISE;

  /* Pの更新 (行列演算を展開) */
  float P[3][3];
  memcpy(P, kf->P, sizeof(P)); // コピー

  float FP[3][3]; /* F * P */
  for (int i = 0; i < 3; i++) {
    FP[0][i] = F00 * P[0][i] + F01 * P[1][i] + F02 * P[2][i];
    FP[1][i] = F10 * P[0][i] + F11 * P[1][i] + F12 * P[2][i];
    FP[2][i] = F20 * P[0][i] + F21 * P[1][i] + F22 * P[2][i];
  }

  /* (F*P) * F^T + Q */
  /* 対角成分にQを足す簡易実装 */
  kf->P[0][0] = (FP[0][0] * F00 + FP[0][1] * F01 + FP[0][2] * F02);
  kf->P[0][1] = (FP[0][0] * F10 + FP[0][1] * F11 + FP[0][2] * F12);
  kf->P[0][2] = (FP[0][0] * F20 + FP[0][1] * F21 + FP[0][2] * F22);

  kf->P[1][0] = (FP[1][0] * F00 + FP[1][1] * F01 + FP[1][2] * F02);
  kf->P[1][1] = (FP[1][0] * F10 + FP[1][1] * F11 + FP[1][2] * F12) + q_acc * dt;
  kf->P[1][2] = (FP[1][0] * F20 + FP[1][1] * F21 + FP[1][2] * F22);

  kf->P[2][0] = (FP[2][0] * F00 + FP[2][1] * F01 + FP[2][2] * F02);
  kf->P[2][1] = (FP[2][0] * F10 + FP[2][1] * F11 + FP[2][2] * F12);
  kf->P[2][2] =
      (FP[2][0] * F20 + FP[2][1] * F21 + FP[2][2] * F22) + q_bias * dt;
}

/* 位置更新 (Measurement Update) */
void kf_update_pos_error(struct kf_state_s *kf, float error_pos) {
  /* H = [1, 0, 0] */
  /* S = P00 + R */
  float S = kf->P[0][0] + VAR_POS_MEAS;
  float invS = 1.0f / S;

  float K[3];
  K[0] = kf->P[0][0] * invS;
  K[1] = kf->P[1][0] * invS;
  K[2] = kf->P[2][0] * invS; /* バイアスも補正される！ */

  /* x = x + K * y */
  kf->x[0] += K[0] * error_pos;
  kf->x[1] += K[1] * error_pos;
  kf->x[2] += K[2] * error_pos;

  /* P = (I - KH) P */
  float P_new[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P_new[i][j] = kf->P[i][j] - K[i] * kf->P[0][j];
    }
  }
  memcpy(kf->P, P_new, sizeof(P_new));
}

/* 速度更新 (Velocity Update) */
void kf_update_vel(struct kf_state_s *kf, float vel_obs) {
  /* H = [0, 1, 0] */
  float y = vel_obs - kf->x[1];
  float S = kf->P[1][1] + VAR_VEL_MEAS;
  float invS = 1.0f / S;

  float K[3];
  K[0] = kf->P[0][1] * invS;
  K[1] = kf->P[1][1] * invS;
  K[2] = kf->P[2][1] * invS;

  kf->x[0] += K[0] * y;
  kf->x[1] += K[1] * y;
  kf->x[2] += K[2] * y;

  float P_new[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P_new[i][j] = kf->P[i][j] - K[i] * kf->P[1][j];
    }
  }
  memcpy(kf->P, P_new, sizeof(P_new));
}

/****************************************************************************
 * Private Functions (Helpers)
 ****************************************************************************/

void save_history(uint32_t timestamp, float *pos) {
  g_history[g_hist_idx].timestamp = timestamp;
  memcpy(g_history[g_hist_idx].pos, pos, sizeof(float) * 3);
  g_hist_idx = (g_hist_idx + 1) % HISTORY_LEN;
}

int get_past_position(uint32_t target_ts, float *out_pos) {
  int i;
  int best_idx = -1;
  uint32_t min_diff = 0xFFFFFFFF;
  for (i = 0; i < HISTORY_LEN; i++) {
    if (g_history[i].timestamp == 0)
      continue;
    uint32_t diff = (target_ts > g_history[i].timestamp)
                        ? (target_ts - g_history[i].timestamp)
                        : (g_history[i].timestamp - target_ts);
    if (diff < min_diff) {
      min_diff = diff;
      best_idx = i;
    }
  }
  if (best_idx == -1 || min_diff > (uint32_t)(0.2f * TS_FREQ))
    return -1;
  memcpy(out_pos, g_history[best_idx].pos, sizeof(float) * 3);
  return 0;
}

static int read_full(int fd, uint8_t *buf, int len) {
  int total = 0;
  while (total < len) {
    int r = read(fd, buf + total, len - total);
    if (r <= 0)
      return -1;
    total += r;
  }
  return total;
}

static int start_sensing(int rate, int adrange, int gdrange, int nfifos) {
  cxd5602pwbimu_range_t range;
  int fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    return -errno;
  ioctl(fd, SNIOC_SSAMPRATE, rate);
  range.accel = adrange;
  range.gyro = gdrange;
  ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  ioctl(fd, SNIOC_ENABLE, 1);
  return fd;
}

static int read_serial_data(int fd, cxd5602pwbimu_data_t *imudata,
                            struct pc_correction_s *corr) {
  uint8_t c;
  int ret;
  struct pollfd fds[2];
  fds[0].fd = fileno(stdin);
  fds[0].events = POLLIN;
  fds[1].fd = fd;
  fds[1].events = POLLIN;
  ret = poll(fds, 2, 10);
  if (ret > 0) {
    if (fds[0].revents & POLLIN) {
      if (read(fds[0].fd, &c, 1) == 1) {
        if (c == 0xAA) {
          uint8_t next;
          if (read(fds[0].fd, &next, 1) == 1 && next == 0x55) {
            uint8_t buf[sizeof(float) * 6 + 1];
            if (read_full(fds[0].fd, buf, sizeof(buf)) == sizeof(buf)) {
              memcpy(corr->vel_now, &buf[0], sizeof(float) * 3);
              memcpy(corr->pos_delayed, &buf[sizeof(float) * 3],
                     sizeof(float) * 3);
              return 3;
            }
          }
        } else if (c == 'z')
          return 2;
        else if (c == 'q')
          return 0;
      }
    }
    if (fds[1].revents & POLLIN) {
      ret = read(fd, imudata, sizeof(*imudata));
      if (ret == sizeof(*imudata))
        return 1;
    }
  }
  return -1;
}

static int read_imudata(int fd, cxd5602pwbimu_data_t *imudata) {
  struct pc_correction_s dummy;
  return read_serial_data(fd, imudata, &dummy);
}

static int drop_50msdata(int fd, int samprate, cxd5602pwbimu_data_t *imu) {
  int cnt = samprate / 20;
  if (cnt == 0)
    cnt = 1;
  while (cnt) {
    read_imudata(fd, imu);
    cnt--;
  }
  return 0;
}

static void init_pospredict(struct pospredict_s *inst) {
  memset(inst, 0, sizeof(*inst));
  inst->posquat[0] = 1.f;
  for (int i = 0; i < 3; i++)
    kf_init(&inst->kf[i]);
}

static void update_posture(float *q, float *gyro, float dt) {
  float mat[4][4], outq[4], norm;
  gyro[0] *= dt;
  gyro[1] *= dt;
  gyro[2] *= dt;
  memset(mat, 0, sizeof(float) * 16);
  mat[1][0] = GH(0);
  mat[2][0] = GH(1);
  mat[3][0] = GH(2);
  mat[0][1] = -GH(0);
  mat[2][1] = -GH(2);
  mat[3][1] = GH(1);
  mat[0][2] = -GH(1);
  mat[1][2] = GH(2);
  mat[3][2] = -GH(0);
  mat[0][3] = -GH(2);
  mat[1][3] = -GH(1);
  mat[2][3] = GH(0);
  memcpy(outq, q, sizeof(float) * 4);
  outq[0] +=
      mat[0][0] * q[0] + mat[0][1] * q[1] + mat[0][2] * q[2] + mat[0][3] * q[3];
  outq[1] +=
      mat[1][0] * q[0] + mat[1][1] * q[1] + mat[1][2] * q[2] + mat[1][3] * q[3];
  outq[2] +=
      mat[2][0] * q[0] + mat[2][1] * q[1] + mat[2][2] * q[2] + mat[2][3] * q[3];
  outq[3] +=
      mat[3][0] * q[0] + mat[3][1] * q[1] + mat[3][2] * q[2] + mat[3][3] * q[3];
  norm = sqrtf(outq[0] * outq[0] + outq[1] * outq[1] + outq[2] * outq[2] +
               outq[3] * outq[3]);
  q[0] = outq[0] / norm;
  q[1] = outq[1] / norm;
  q[2] = outq[2] / norm;
  q[3] = outq[3] / norm;
}

/* 欠落していたcalibration関数を追加しました */
static void calibration(int fd, cxd5602pwbimu_data_t *imu, float *posquat,
                        float *gravity, float *gbias) {
  int i, j;
  int cnt;
  float accav[3];
  memset(gbias, 0, sizeof(float) * 3);
  memset(accav, 0, sizeof(float) * 3);

  printf("Calibration will start in 3sec..\n T: ");
  for (i = 0; i < 3; i++) {
    printf("%d ", i - 3);
    fflush(stdout);
    for (j = 0; j < SAMPLERATE; j++) {
      read_imudata(fd, imu);
    }
  }

  printf("\n Calibration for 10sec..\n");
  cnt = 0;
  for (j = 0; j < 10; j++) {
    for (i = 0; i < SAMPLERATE; i++) {
      if (read_imudata(fd, imu) == 1) {
        cnt++;
        accav[0] += imu->ax;
        accav[1] += imu->ay;
        accav[2] += imu->az;
        gbias[0] += imu->gx;
        gbias[1] += imu->gy;
        gbias[2] += imu->gz;
      }
    }
    printf("%d", 9 - j);
    fflush(stdout);
  }

  accav[0] /= (float)cnt;
  accav[1] /= (float)cnt;
  accav[2] /= (float)cnt;
  postureByAccel(posquat, accav[0], accav[1], accav[2], 0.f);
  *gravity =
      sqrtf(accav[0] * accav[0] + accav[1] * accav[1] + accav[2] * accav[2]);

  gbias[0] /= (float)cnt;
  gbias[1] /= (float)cnt;
  gbias[2] /= (float)cnt;

  printf("\nG:%1.2f GB:(%2.2f, %2.2f, %2.2f)\n", *gravity, gbias[0], gbias[1],
         gbias[2]);
}

static void quat_to_rotmat(float *q, float (*rot)[3]) {
  rot[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  rot[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
  rot[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
  rot[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
  rot[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
  rot[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
  rot[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  rot[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
  rot[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
}

static void kf_prediction_step(struct pospredict_s *inst,
                               cxd5602pwbimu_data_t *imu) {
  int i;
  float tsdiff, adj_acc[3], rotM[3][3];
  tsdiff = ((float)(imu->timestamp - inst->last_ts)) / ((float)TS_FREQ);
  inst->last_ts = imu->timestamp;
  inst->calibtime += tsdiff;
  imu->gx -= inst->gbias[0];
  imu->gy -= inst->gbias[1];
  imu->gz -= inst->gbias[2];
  update_posture(inst->posquat, &imu->gx, tsdiff);
  quat_to_rotmat(inst->posquat, rotM);
  memset(adj_acc, 0, sizeof(adj_acc));
  for (i = 0; i < 3; i++) {
    adj_acc[i] += rotM[i][0] * imu->ax;
    adj_acc[i] += rotM[i][1] * imu->ay;
    adj_acc[i] += rotM[i][2] * imu->az;
  }
  adj_acc[2] -= inst->gravity;

  /* KF Predict (3軸) */
  for (i = 0; i < 3; i++) {
    kf_predict(&inst->kf[i], adj_acc[i], tsdiff);
  }
}

int main(int argc, FAR char *argv[]) {
  int i, fd, dispcnt = 0;
  cxd5602pwbimu_data_t imu;
  float e[3];
  static int hist_decim_cnt = 0;

  init_pospredict(&g_inst);
  fd = start_sensing(SAMPLERATE, ARANGE, GRANGE, 1);
  drop_50msdata(fd, SAMPLERATE, &imu);
  calibration(fd, &imu, g_inst.posquat, &g_inst.gravity, g_inst.gbias);
  sleep(1);
  for (i = 0; i < 10; i++)
    read_imudata(fd, &imu);
  g_inst.last_ts = imu.timestamp;

  while (1) {
    struct pc_correction_s pc_data;
    int ret = read_serial_data(fd, &imu, &pc_data);

    if (ret == 3) {
      /* === 補正処理 === */
      float pc_vel[3];
      for (i = 0; i < 3; i++)
        pc_vel[i] = pc_data.vel_now[i];
#if INVERT_PC_AXIS
      for (i = 0; i < 3; i++)
        pc_vel[i] = -pc_vel[i];
#endif
      /* 1. 速度観測による更新 (Velocity Update) */
      for (i = 0; i < 3; i++)
        kf_update_vel(&g_inst.kf[i], pc_vel[i]);

      /* 2. 位置誤差による更新 (Position Update) */
      float my_past_pos[3];
      uint32_t target_ts = imu.timestamp - (10 * TS_FREQ);

      if (get_past_position(target_ts, my_past_pos) == 0) {
        float pc_pos[3];
        memcpy(pc_pos, pc_data.pos_delayed, sizeof(float) * 3);
#if INVERT_PC_AXIS
        for (i = 0; i < 3; i++)
          pc_pos[i] = -pc_pos[i];
#endif
        float error_pos[3];
        for (i = 0; i < 3; i++)
          error_pos[i] = pc_pos[i] - my_past_pos[i];

        /* 位置誤差で状態（バイアス含む）を更新！ */
        for (i = 0; i < 3; i++)
          kf_update_pos_error(&g_inst.kf[i], error_pos[i]);

        printf("AugKF Upd! Err:%.3f Bias:%.3f\n", error_pos[0],
               g_inst.kf[0].x[2]);
      }
    } else if (ret == 1) {
      kf_prediction_step(&g_inst, &imu);

      float current_pos[3], current_vel[3];
      for (i = 0; i < 3; i++) {
        current_pos[i] = g_inst.kf[i].x[0];
        current_vel[i] = g_inst.kf[i].x[1];
      }

      if (hist_decim_cnt++ >= (SAMPLERATE / HISTORY_FREQ)) {
        save_history(imu.timestamp, current_pos);
        hist_decim_cnt = 0;
      }
      if (dispcnt >= (SAMPLERATE / 10)) {
        quaternion2euler(g_inst.posquat, e);
        printf("EST: X:%2.2f Y:%2.2f Z:%2.2f V:%2.2f,%2.2f,%2.2f R:%2.2f "
               "Y:%2.2f P:%2.2f\n",
               current_pos[0], current_pos[1], current_pos[2], current_vel[0],
               current_vel[1], current_vel[2], RAD2DEG(e[0]), RAD2DEG(e[1]),
               RAD2DEG(e[2]));
        dispcnt = 0;
      }
      dispcnt++;
    } else if (ret == 2) {
      printf("==== Reset ====\n");
      for (i = 0; i < 3; i++)
        kf_init(&g_inst.kf[i]);
      g_inst.calibtime = 0.f;
    } else if (ret == 0) {
      goto endapp;
    }
  }

endapp:
  close(fd);
  return 0;
}