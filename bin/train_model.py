import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
import sys
import os

# データセットの確認
if not os.path.exists('dataset_all.csv'):
    print("Error: dataset_all.csv が見つかりません。")
    # ファイルがない場合は空のファイルを作るか、エラーで終了
    sys.exit(1)

# データの読み込み
df = pd.read_csv('dataset_all.csv', header=None)

print("読み込みok")

# カラム名を設定 
# 0:入力キレ, 1:入力大きさ, 3~7:末端部位の移動距離の平均, 8:ねじれ平均, 9:正解kire, 10~16:正解furi[0]~[6]
df.columns = [
    'input_sharpness', 'input_magnitude',
    'input_right_foot_dist_ave','input_left_foot_dist_ave',
    'input_right_hand_dist_ave','input_left_hand_dist_ave',
    'input_head_dist_ave',
    'input_ChestVal',
    'input_moving_ratio',
    'input_interaction',
    'target_kire',
    'target_furi_0', 'target_furi_1', 'target_furi_2', 'target_furi_3',
    'target_furi_4', 'target_furi_5', 'target_furi_6',
    'target_bz_1x', 'target_bz_1y',
    'target_bz_2x', 'target_bz_2y'
]

X_raw = df[['input_sharpness', 'input_magnitude',
        'input_right_foot_dist_ave','input_left_foot_dist_ave',
        'input_right_hand_dist_ave','input_left_hand_dist_ave',
        'input_head_dist_ave',
        'input_ChestVal',
        'input_moving_ratio',
        'input_interaction']]

# 標準化のインスタンスを作成
scaler = StandardScaler()
# データの平均と標準偏差を計算し、Xを標準化（平均0, 分散1）に変換
X_scaled = scaler.fit_transform(X_raw)

# 学習と書き出し
with open('model_params.txt', 'w') as f:
    # kire の学習
    model_kire = LinearRegression()
    model_kire.fit(X_scaled, df['target_kire'])
    # 係数10つと切片1つを書き出す (スペース区切り)
    f.write(f"{model_kire.coef_[0]} {model_kire.coef_[1]} {model_kire.coef_[2]} {model_kire.coef_[3]} \
            {model_kire.coef_[4]} {model_kire.coef_[5]} {model_kire.coef_[6]} {model_kire.coef_[7]} \
            {model_kire.coef_[8]} {model_kire.coef_[9]} \
            {model_kire.intercept_}\n")

    # furi[0]～furi[6] の学習
    for i in range(7):
        target_col = f'target_furi_{i}'
        model_furi = LinearRegression()
        model_furi.fit(X_scaled, df[target_col])
        f.write(f"{model_furi.coef_[0]} {model_furi.coef_[1]} {model_furi.coef_[2]} {model_furi.coef_[3]} \
                {model_furi.coef_[4]} {model_furi.coef_[5]} {model_furi.coef_[6]} {model_furi.coef_[7]} \
                {model_furi.coef_[8]} {model_furi.coef_[9]} \
                {model_furi.intercept_}\n")

    # ベジェ曲線の制御点の学習
    for i in range(4):
        target_col = f'target_bz_{i}'
        model_bz = LinearRegression()
        model_bz.fit(X_scaled, df[target_col])
        f.write(f"{model_bz.coef_[0]} {model_bz.coef_[1]} {model_bz.coef_[2]} {model_bz.coef_[3]} \
                {model_bz.coef_[4]} {model_bz.coef_[5]} {model_bz.coef_[6]} {model_bz.coef_[7]} \
                {model_bz.coef_[8]} {model_bz.coef_[9]} \
                {model_bz.intercept_}\n")

    # C++側で再現するために、各項目の「平均」を一行に書き出します
    f.write(" ".join(map(str, scaler.mean_)) + "\n")
    # 次の行に、各項目の「標準偏差」を書き出します
    f.write(" ".join(map(str, scaler.scale_)) + "\n")

print("学習完了: model_params.txt を更新しました。")