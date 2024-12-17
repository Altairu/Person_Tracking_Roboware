import pandas as pd
import matplotlib.pyplot as plt

# サンプリング間隔（時間ステップ）
TIMESTEP = 0.1

# ファイルパス
pn_files = ["PN_data1.csv", "PN_data2.csv"]
mpn_files = ["MPN_data1.csv", "MPN_data2.csv"]

def analyze_data(file_list, label):
    """視野内、ズレ、加速度を解析"""
    results = []
    for i, file_path in enumerate(file_list):
        # データ読み込み
        df = pd.read_csv(file_path)

        # 視野内のフレーム数の割合 (PersonOffsetが画面中心から一定以内)
        in_frame = ((df["PersonOffset"] >= -320) & (df["PersonOffset"] <= 320)).sum()
        total_frames = len(df)
        in_frame_rate = in_frame / total_frames

        # 横オフセット (ズレ)
        offset = df["PersonOffset"].abs()
        mean_offset = offset.mean()
        std_offset = offset.std()

        # 加速度 (TargetRight と TargetLeft の平均速度から加速度計算)
        avg_velocity = (df["TargetRight"] + df["TargetLeft"]) / 2
        acceleration = avg_velocity.diff() / TIMESTEP
        mean_accel = acceleration.mean()
        std_accel = acceleration.std()

        # 結果を追加
        results.append({
            "Experiment": f"{label}_data{i+1}",
            "InFrameRate": in_frame_rate,
            "MeanOffset": mean_offset,
            "StdOffset": std_offset,
            "MeanAccel": mean_accel,
            "StdAccel": std_accel
        })

    return pd.DataFrame(results)

def plot_results(results, title, ylabel, metrics):
    """結果をグラフでプロット"""
    plt.figure(figsize=(10, 6))
    for metric in metrics:
        plt.plot(results["Experiment"].to_numpy(), results[metric].to_numpy(), label=metric, marker="o")
    plt.title(title)
    plt.xlabel("Experiments")
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.show()


# データ解析
pn_results = analyze_data(pn_files, "PN")
mpn_results = analyze_data(mpn_files, "MPN")

# 結果の表示
print("\nPN Results:")
print(pn_results)
print("\nMPN Results:")
print(mpn_results)

# 結果を結合して比較
combined_results = pd.concat([pn_results, mpn_results], ignore_index=True)

# グラフ表示
plot_results(combined_results, "In-Frame Rate Comparison", "In-Frame Rate", ["InFrameRate"])
plot_results(combined_results, "Mean Offset Comparison", "Offset (px)", ["MeanOffset", "StdOffset"])
plot_results(combined_results, "Acceleration Comparison", "Acceleration (m/s²)", ["MeanAccel", "StdAccel"])
