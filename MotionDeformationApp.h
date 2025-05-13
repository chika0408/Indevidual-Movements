/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作変形アプリケーション
**/

#ifndef  _MOTION_DEFORMATION_APP_H_
#define  _MOTION_DEFORMATION_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "InverseKinematicsCCDApp.h"
#include "HumanBody.h"
#include <vector>


// プロトタイプ宣言
class  Timeline;

//
// 末端部分の移動距離の合計の情報
//
struct DistanceParam
{
	// 末端部分の移動距離の合計
	float distanceadd;

	// distanceaddの要素が閾値を超えているかどうかを保存する
	// 0のときは止まっている、1のときは動いている
	int movecheck;

	// 動作が開始したかどうかを判断する
	// 動作が開始しているときはtrue
	bool move_start;

	// 動作が動いているかどうかの閾値
	float move_amount;
};


//
// タイムワーピングの情報
//
struct TimeWarpingParam
{
	// 姿勢変形の始まり(時間表記)
	float        warp_in_duration_time;

	// 姿勢変形前のキー時刻（時間表記）
	float		 warp_key_time;

	// 姿勢変形の終わり(時間表記)
	float        warp_out_duration_time;

	// 姿勢変形後のキー時刻（時間表記）
	float		 after_key_time;
};


//
//  動作変形（動作ワーピング）の情報（動作中の１つのキー時刻の姿勢を変形）
//
struct  MotionWarpingParam
{
	// 姿勢変形を適用するキー時刻
	float        key_time;

	// 変形前のキー時刻の姿勢
	Posture      org_pose;

	// 変形後のキー時刻の姿勢
	Posture      key_pose;

	// 姿勢変形の前後のブレンド時間
	float        blend_in_duration;
	float        blend_out_duration;
};


//
//  動作変形アプリケーションクラス
//
class  MotionDeformationApp : public InverseKinematicsCCDApp
{
  protected:
	// 動作＋動作変形情報
	
	// 動作変形を適用する動作データ
	Motion *           motion;

	// 末端部分の移動距離の合計の情報
	vector<DistanceParam> distanceinfo;

	// 動作変形(動作ワーピング)情報
	MotionWarpingParam deformation;

	// 動作変形(タイムワーピング)情報
	TimeWarpingParam timewarp_deformation;

	// タイムワーピング実行後の前フレームの時間
	float    before_frame_time;

	// フリレベル
	float		furi;

	// キレレベル
	float		kire;

	// ２つ目の動作データ
	Motion*		second_motion;

	// ２つ目の動作データの姿勢
	Posture*	second_curr_posture;

	// モーションワーピングの重み表示用
	float		weight;


  protected:
	// 動作再生のための変数

	// 変形前のキャラクタの姿勢
	Posture *          org_posture;

	// 変形後のキャラクタの姿勢
	Posture *          deformed_posture;

	// アニメーション再生中かどうかを表すフラグ
	bool               on_animation;

	// 動作の再生時間
	float              animation_time;

	// 動作の再生速度
	float              animation_speed;

	// 現在の表示フレーム番号
	int                frame_no;

  protected:
	// 画面描画のための変数

	// 動作変形前の姿勢・キー姿勢の描画フラグ
	bool               draw_original_posture;
	bool               draw_key_posture;
	bool               draw_postures_side_by_side;

	// タイムライン描画機能
	Timeline *         timeline;


  public:
	// コンストラクタ
	MotionDeformationApp();

	// デストラクタ
	virtual ~MotionDeformationApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// ウィンドウサイズ変更
	virtual void  Reshape( int w, int h );

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	//  マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );

  protected:
	// 内部処理

	// 入力動作の初期化
	void  InitMotion( int no );

	// 動作変形情報の初期化
	void  InitParameter();

	// 動作変形情報にもとづくタイムラインの初期化
	void  InitTimeline( Timeline * timeline, const Motion & motion, const MotionWarpingParam & motiondeform, TimeWarpingParam & timedeform, float curr_time );

	// BVH動作ファイルの読み込み、骨格・姿勢の初期化
	void  LoadBVH( const char * file_name );

	// ２つ目の動作ファイルの読み込み
	void  LoadSecondBVH(const char* file_name);

	// 変形後の動作をBVH動作ファイルとして保存
	void  SaveDeformedMotionAsBVH( const char * file_name );

};

//
// 末端部位の移動距離の合計を求める処理
//

// 末端部位の移動距離の合計の情報の初期化
void InitDistanceParameter(vector<DistanceParam> & param);

// 末端部位の移動距離を測定
void CheckDistance(const Motion& motion, vector<DistanceParam> & param, const char ** segment_names = NULL);

// 応急処置
int Getseg(int i);

//
//  動作変形情報にもとづく動作変形処理（タイムワーピング）
//

// 動作変形（タイムワーピング）の情報の初期化・更新
void  InitTimeDeformationParameter(vector<DistanceParam>& distance, TimeWarpingParam& param, Motion motion, float kire);

// 動作変形（タイムワーピング）の情報の初期化・更新
void  InitTimeDeformationParameter(float now_time, vector<DistanceParam>& distance, TimeWarpingParam& param, Motion motion, float kire);

// 動作変形（タイムワーピング）の情報の更新
void ReTimeDeformationParameter(float warp_in_duration, float warp_out_duration, TimeWarpingParam& param);

// タイムワーピングの適用後の姿勢計算
void ApplyTimeWarping(float now_time, TimeWarpingParam& deform, const Motion& input_motion, float& before_frame_time, Posture& output_pose);

// タイムワーピング実行後の時刻を取得
float Warping(float now_time, TimeWarpingParam& deform);


//
//  動作変形情報にもとづく動作変形処理（モーションワーピング）
//

// 動作変形（動作ワーピング）の情報の初期化
void  InitDeformationParameter( const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	MotionWarpingParam & deform );

// 動作変形（動作ワーピング）の情報の初期化
void  InitDeformationParameter( const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	int base_joint_no, int ee_joint_no, Point3f ee_joint_translation, 
	MotionWarpingParam & deform );

// 動作変形（動作ワーピング）の情報の初期化・更新
void  InitDeformationParameter(
	float now_time, vector<DistanceParam> distance, MotionWarpingParam& param, TimeWarpingParam time_param, Motion& motion, float furi);

// 動作変形（動作ワーピング）の適用後の動作を生成
Motion *  GenerateDeformedMotion( const MotionWarpingParam & deform, const Motion & motion );

// 動作変形（動作ワーピング）の適用後の姿勢の計算
float  ApplyMotionDeformation( float time, const MotionWarpingParam & deform, Motion& motion, Posture & input_pose, TimeWarpingParam time_param, Posture & output_pose );

// 動作ワーピングの姿勢変形（２つの姿勢の差分（dest - src）に重み ratio をかけたものを元の姿勢 org に加える ）
void  PostureWarping( const Posture & org, const Posture & src, const Posture & dest, float ratio, Posture & p );



#endif // _MOTION_DEFORMATION_APP_H_
