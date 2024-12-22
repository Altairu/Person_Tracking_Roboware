import numpy as np
import matplotlib.pyplot as plt

def dynamic_kd(base_kd, omega, a=1.0):
    """
    角速度に応じて微分ゲインを動的に変化させる関数
    Args:
        base_kd (float): 基本微分ゲイン (最大値)
        omega (float): 角速度 (rad/s)
        a (float): 調整パラメータ (変化の緩やかさ)
    Returns:
        float: 動的に変化した微分ゲイン
    """
    omega_abs = abs(omega)
    kd_dynamic = base_kd * (1 - np.exp(-a * omega_abs)) / (1 + np.exp(-a * omega_abs))
    return kd_dynamic

# 設定
base_kd = 10.0  # 基本微分ゲイン
omegas = np.linspace(-100, 100, 200)  # 角速度範囲

# 異なるパラメータaで動的Kdを計算
a_values = [0.05,0.1,0.2, 0.5, 1.0, 2.0]  # 小さいaほど変化が緩やか
kd_results = {a: [dynamic_kd(base_kd, omega, a) for omega in omegas] for a in a_values}

# グラフの描画
plt.figure(figsize=(10, 6))
for a, kd in kd_results.items():
    plt.plot(omegas, kd, label=f"a = {a}")
plt.title("Dynamic Kd Based on Angular Velocity |ω|")
plt.xlabel("Angular Velocity ω (rad/s)")
plt.ylabel("Dynamic Kd")
plt.grid()
plt.legend()
plt.show()
