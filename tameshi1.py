import numpy as np
import matplotlib.pyplot as plt
import matplotlib

# 日本語フォント設定 (Noto Sans CJK JPを使用)
matplotlib.rcParams['font.family'] = 'Noto Sans CJK JP'

def 動的微分ゲイン(base_kd, omega, a=1.0):
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
基本微分ゲイン = 10.0  # 基本微分ゲイン
角速度範囲 = np.linspace(-100, 100, 200)  # 角速度範囲

# 異なるパラメータaで動的Kdを計算
aの値 = [0.05, 0.1, 0.2, 0.5]  # 小さいaほど変化が緩やか
結果 = {a: [動的微分ゲイン(基本微分ゲイン, omega, a) for omega in 角速度範囲] for a in aの値}

# グラフの描画
plt.figure(figsize=(10, 6))
for a, kd in 結果.items():
    plt.plot(角速度範囲, kd, label=f"a = {a}")
plt.title("角速度 |ω| に基づく動的微分ゲイン")
plt.xlabel("角速度 ω (rad/s)")
plt.ylabel("動的微分ゲイン")
plt.grid()
plt.legend()

# PDFとして保存 (英語のファイル名)
plt.savefig("dynamic_gain.pdf", format="pdf", bbox_inches="tight")
plt.show()

