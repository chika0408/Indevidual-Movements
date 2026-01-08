import pandas as pd
from sklearn.linear_model import LinearRegression
import sys
import os

# データセットの確認
if not os.path.exists('dataset_all.csv'):
    print("Error: dataset_all.csv が見つかりません。")
    # ファイルがない場合は空のファイルを作るか、エラーで終了
    sys.exit(1)

# データの読み込み
df = pd.read_csv('dataset_all.csv', header=None)

# カラム名を設定 
# 0:入力キレ, 1:入力大きさ, 3~7:末端部位の移動距離の平均, 8:ねじれ平均, 9:正解kire, 10~16:正解furi[0]~[6]
df.columns = [
    'input_sharpness', 'input_magnitude',
    'input_right_foot_dist_ave','input_left_foot_dist_ave',
    'input_right_hand_dist_ave','input_left_hand_dist_ave',
    'input_head_dist_ave',
    'input_ChestVal',
    'target_kire',
    'target_furi_0', 'target_furi_1', 'target_furi_2', 'target_furi_3',
    'target_furi_4', 'target_furi_5', 'target_furi_6'
]

X = df[['input_sharpness', 'input_magnitude',
        'input_right_foot_dist_ave','input_left_foot_dist_ave',
        'input_right_hand_dist_ave','input_left_hand_dist_ave',
        'input_head_dist_ave',
        'input_ChestVal']]

# 学習と書き出し
with open('model_params.txt', 'w') as f:
    # kire の学習
    model_kire = LinearRegression()
    model_kire.fit(X, df['target_kire'])
    # 係数8つと切片1つを書き出す (スペース区切り)
    f.write(f"{model_kire.coef_[0]} {model_kire.coef_[1]} {model_kire.coef_[2]} {model_kire.coef_[3]} \
            {model_kire.coef_[4]} {model_kire.coef_[5]} {model_kire.coef_[6]} {model_kire.coef_[7]} \
            {model_kire.intercept_}\n")

    # furi[0]～furi[6] の学習
    for i in range(7):
        target_col = f'target_furi_{i}'
        model_furi = LinearRegression()
        model_furi.fit(X, df[target_col])
        f.write(f"{model_furi.coef_[0]} {model_furi.coef_[1]} {model_furi.coef_[2]} {model_furi.coef_[3]} \
                {model_furi.coef_[4]} {model_furi.coef_[5]} {model_furi.coef_[6]} {model_furi.coef_[7]} \
                {model_furi.intercept_}\n")

print("学習完了: model_params.txt を更新しました。")