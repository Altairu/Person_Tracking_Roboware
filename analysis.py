import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib.font_manager as fm
# 日本語フォントを明示的に指定
font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"  # 適切なフォントパスを指定
font_prop = fm.FontProperties(fname=font_path)
rcParams['font.family'] = font_prop.get_name()
rcParams['axes.unicode_minus'] = False  # マイナス記号の表示修正

# データフレームの作成
pn_results = pd.DataFrame({
    "Experiment": ["PN_data1", "PN_data2"],
    "InFrameRate": [1.0, 1.0],
    "MeanOffset": [41.861897, 59.888889],
    "StdOffset": [54.028319, 64.740366],
    "MeanAccel": [abs(-29.083331), abs(-128.507793)],  # 絶対値を取る
    "StdAccel": [29109.584411, 27609.337063],
})

mpn_results = pd.DataFrame({
    "Experiment": ["MPN_data1", "MPN_data2"],
    "InFrameRate": [1.0, 1.0],
    "MeanOffset": [102.423611, 36.154185],
    "StdOffset": [57.045868, 51.269086],
    "MeanAccel": [abs(202.613259), abs(55.077258)],  # 絶対値を取る
    "StdAccel": [27873.365387, 11523.682647],
})

gs_mpn_results = pd.DataFrame({
    "Experiment": ["GS-MPN_data1", "GS-MPN_data2"],
    "InFrameRate": [1.0, 1.0],
    "MeanOffset": [66.287293, 48.940909],
    "StdOffset": [69.297506, 69.981945],
    "MeanAccel": [abs(-101.004165), abs(-50.113902)],  # 絶対値を取る
    "StdAccel": [11177.160097, 21241.415990],
})

# 結果を統合
combined_results = pd.concat([pn_results, mpn_results, gs_mpn_results], ignore_index=True)

# グラフ作成関数
def plot_results(results, title, ylabel, metrics):
    """結果をグラフでプロット"""
    plt.figure(figsize=(10, 6))
    for metric in metrics:
        plt.plot(results["Experiment"].to_numpy(), results[metric].to_numpy(), label=metric, marker="o")
    plt.title(title)
    plt.xlabel("実験")
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.show()

# グラフ表示
plot_results(combined_results, "ズレの比較", "ズレ (px)", ["MeanOffset", "StdOffset"])
plot_results(combined_results, "加速度の比較（絶対値）", "加速度 (m/s²)", ["MeanAccel", "StdAccel"])

# 表示
print("PN, MPN, GS-MPN 結果（統合）:")
print(combined_results)
