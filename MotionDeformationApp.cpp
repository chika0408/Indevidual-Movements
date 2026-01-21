/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作変形アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "Timeline.h"
#include "MotionDeformationApp.h"
#include "HumanBody.h"
#include <vector>
#include <algorithm>

// csvファイル作成のため
#include <fstream>
#include <iostream>

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <numeric> // std::accumulate用
#include <cmath>   // std::pow用

// windowsの機能利用のため
#ifdef _WIN32
#include <windows.h>
#endif

//
//  コンストラクタ
//
MotionDeformationApp::MotionDeformationApp() : InverseKinematicsCCDApp()
{
	app_name = "Motion Deformation Base";

	motion = NULL;

	org_posture = NULL;
	deformed_posture = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	frame_no = 0;
	before_frame_time = 0.0f;

	second_motion = NULL;
	second_curr_posture = NULL;

	draw_original_posture = false;
	draw_postures_side_by_side = false;
	timeline = NULL;
}


//
//  デストラクタ
//
MotionDeformationApp::~MotionDeformationApp()
{
	if ( motion )
		delete  motion;
	if ( curr_posture && curr_posture->body )
		delete  curr_posture->body;
	if ( curr_posture )
		delete  curr_posture;
	if ( org_posture )
		delete  org_posture;
	if ( deformed_posture )
		delete  deformed_posture;
	if ( timeline )
		delete  timeline;

	if (second_curr_posture)
		delete  second_curr_posture;
	if (second_motion)
		delete  second_motion;

}

//
// 骨格に含まれる名前をチェックして、適切な部位名リストを作成する関数
//

void  GetAdaptiveSegmentNames(const Skeleton* skeleton, const char** names)
{
	// デフォルト（Char00系）を設定
	names[SEG_R_FOOT] = "RightFoot";
	names[SEG_L_FOOT] = "LeftFoot";
	names[SEG_R_HAND] = "RightHand";
	names[SEG_L_HAND] = "LeftHand";
	names[SEG_PELVIS] = "Hips";
	names[SEG_CHEST] = "Spine3";//chest
	names[SEG_HEAD] = "Neck";

	if (!skeleton) return;

	// 現在の設定名が見つからなければ、代替案(alt_name)を探して設定する
	auto TryReplace = [&](int seg_idx, const char* alt_name) {
		bool found = false;
		// 現在の設定名があるかチェック
		for (int i = 0; i < skeleton->num_segments; i++) {
			if (strstr(skeleton->segments[i]->name.c_str(), names[seg_idx])) {
				found = true;
				break;
			}
		}
		// なければ代替案をチェック
		if (!found) {
			for (int i = 0; i < skeleton->num_segments; i++) {
				if (strstr(skeleton->segments[i]->name.c_str(), alt_name)) {
					names[seg_idx] = alt_name; // 代替案を採用
					break;
				}
			}
		}
		};

	// 一般的なBVH（sample_walking2など）向けの代替名をチェック
	TryReplace(SEG_R_FOOT, "RightAnkle");
	TryReplace(SEG_L_FOOT, "LeftAnkle");
	TryReplace(SEG_R_HAND, "RightWrist");
	TryReplace(SEG_L_HAND, "LeftWrist");
	TryReplace(SEG_CHEST, "Chest");
	TryReplace(SEG_CHEST, "Spine"); // ChestもなければSpineを試す
}

//
// 骨格に含まれる名前をチェックして、適切な関節名リストを作成する関数
//
void GetAdaptiveJointNames(const Skeleton* skeleton, const char** names)
{
	// デフォルト設定
	names[JOI_R_SHOULDER] = "RightArm";
	names[JOI_L_SHOULDER] = "LeftArm";
	names[JOI_R_ELBOW] = "RightForeArm";
	names[JOI_L_ELBOW] = "LeftForeArm";
	names[JOI_R_WRIST] = "RightHand";
	names[JOI_L_WRIST] = "LeftHand";
	names[JOI_R_HIP] = "RightUpLeg";
	names[JOI_L_HIP] = "LeftUpLeg";
	names[JOI_R_KNEE] = "RightLeg";
	names[JOI_L_KNEE] = "LeftLeg";
	names[JOI_R_ANKLE] = "RightFoot";
	names[JOI_L_ANKLE] = "LeftFoot";
	names[JOI_BACK] = "Spine";
	names[JOI_NECK] = "Neck";

	if (!skeleton) return;

	// ヘルパー: 代替案を探す
	auto TryReplace = [&](int joi_idx, const char* alt_name) {
		bool found = false;
		for (int i = 0; i < skeleton->num_joints; i++) {
			if (strstr(skeleton->joints[i]->name.c_str(), names[joi_idx])) {
				found = true;
				break;
			}
		}
		if (!found) {
			for (int i = 0; i < skeleton->num_joints; i++) {
				if (strstr(skeleton->joints[i]->name.c_str(), alt_name)) {
					names[joi_idx] = alt_name;
					break;
				}
			}
		}
		};

	// 一般的なBVH（sample_walking2など）向けの代替名チェック
	// 肩・腕周り
	TryReplace(JOI_R_SHOULDER, "RightShoulder"); // "RightArm" がなければ "RightShoulder"
	TryReplace(JOI_R_SHOULDER, "RightCollar");   // それもなければ "RightCollar"
	TryReplace(JOI_L_SHOULDER, "LeftShoulder");
	TryReplace(JOI_L_SHOULDER, "LeftCollar");

	// 肘・前腕
	TryReplace(JOI_R_ELBOW, "RightElbow");       // "RightForeArm" がなければ "RightElbow"
	TryReplace(JOI_L_ELBOW, "LeftElbow");

	// 手首
	TryReplace(JOI_R_WRIST, "RightWrist");       // "RightHand" がなければ "RightWrist"
	TryReplace(JOI_L_WRIST, "LeftWrist");

	// 腰・腿
	TryReplace(JOI_R_HIP, "RightHip");           // "RightUpLeg" がなければ "RightHip"
	TryReplace(JOI_L_HIP, "LeftHip");

	// 膝・すね
	TryReplace(JOI_R_KNEE, "RightKnee");         // "RightLeg" がなければ "RightKnee"
	TryReplace(JOI_L_KNEE, "LeftKnee");

	// 足首
	TryReplace(JOI_R_ANKLE, "RightAnkle");       // "RightFoot" がなければ "RightAnkle"
	TryReplace(JOI_L_ANKLE, "LeftAnkle");

	// 背骨・胸
	TryReplace(JOI_BACK, "Chest");               // "Spine" がなければ "Chest"
	TryReplace(JOI_BACK, "Spine1");
}


//
//  初期化
//
void  MotionDeformationApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// 入力動作の初期化
	InitMotion( 0 );
	// Distanceinfo構造体の初期化
	InitDistanceParameter(distanceinfo);

	// 姿勢描画の設定
	draw_original_posture = true;
	draw_key_posture = true;
	draw_postures_side_by_side = false;

	// 解析を行う動作データの骨格の、主要体節の名前を設定
	const char * primary_segment_names[NUM_PRIMARY_SEGMENTS];

	if (motion && motion->body) {
		GetAdaptiveSegmentNames(motion->body, primary_segment_names);
	}
	else {
		// フォールバック
		primary_segment_names[SEG_R_FOOT] = "RightFoot";
		primary_segment_names[SEG_L_FOOT] = "LeftFoot";
		primary_segment_names[SEG_R_HAND] = "RightHand";
		primary_segment_names[SEG_L_HAND] = "LeftHand";
		primary_segment_names[SEG_PELVIS] = "Hips";
		primary_segment_names[SEG_CHEST] = "Spine3";
		primary_segment_names[SEG_HEAD] = "Neck";
	}

	// 末端部位の移動距離の合計をフレーム毎に配列として出力
	CheckDistance(*motion, distanceinfo, model_param, primary_segment_names);

	// HumanBodyのセットアップ
	if (motion && motion->body) {
		// 既存のmotion->bodyからHumanBodyを作成
		my_human_body = new HumanBody(motion->body);

		for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) {
			my_human_body->SetPrimarySegment((PrimarySegmentType)i, primary_segment_names[i]);
		}
	}
	else {
		my_human_body = NULL;
	}

	// 前フレーム座標の初期化
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) {
		prev_segment_positions[i] = Point3f(0, 0, 0);
	}

	// 【追加】キャッシュ変数の初期化
	cached_segment_pos_offsets.clear();
	cached_segment_rot_offsets.clear();
	// 最初のフレームで必ず計算が走るように、ありえない値で初期化
	for (int i = 0; i < 7; i++) prev_furi[i] = -999.9f;

	// フリレベル・キレレベルの初期設定
	furi[0] = 1.0f;
	furi[1] = 1.0f;
	furi[2] = 1.0f;
	furi[3] = 1.0f;
	furi[4] = 1.0f;
	furi[5] = 1.0f;
	furi[6] = 1.0f;
	kire = 1.0f;

	input_furi = -10.0f;
	input_kire = -10.0f;

	// 交差項の設定
	model_param.interaction = input_furi * input_kire;

	// 編集中のレベルの設定
	selected_param = 0;

	// ねじれの計算
	model_param.ChestVal = CalcChestVal(motion);

	// 動作変形情報の初期化
	//InitParameter();

	//　Python学習と結果読み込み
	//int ret = system("python train_model.py");
	//if (ret == 0) {
	//	std::ifstream infile("model_params.txt");
	//	if (infile.is_open()) {
	//		// kire: 重み10つ + 切片
	//		infile >> model_param.params_kire[0] >> model_param.params_kire[1]
	//			>> model_param.params_kire[2] >> model_param.params_kire[3]
	//			>> model_param.params_kire[4] >> model_param.params_kire[5]
	//			>> model_param.params_kire[6] >> model_param.params_kire[7]
	//			>> model_param.params_kire[8] >> model_param.params_kire[9]
	//			>> model_param.params_kire[10];

	//		// furi: 重み10つ + 切片
	//		for (int i = 0; i < 7; i++) {
	//			infile >> model_param.params_furi[i][0] >> model_param.params_furi[i][1]
	//				>> model_param.params_furi[i][2] >> model_param.params_furi[i][3]
	//				>> model_param.params_furi[i][4] >> model_param.params_furi[i][5]
	//				>> model_param.params_furi[i][6] >> model_param.params_furi[i][7]
	//				>> model_param.params_furi[i][8] >> model_param.params_furi[i][9]
	//				>> model_param.params_furi[i][10];
	//		}
	// 
	//		// bezier: 重み10つ + 切片
	//		for (int i = 0; i < 4; i++) {
	//			infile >> model_param.params_bezier[i][0] >> model_param.params_bezier[i][1]
	//				>> model_param.params_bezier[i][2] >> model_param.params_bezier[i][3]
	//				>> model_param.params_bezier[i][4] >> model_param.params_bezier[i][5]
	//				>> model_param.params_bezier[i][6] >> model_param.params_bezier[i][7]
	//				>> model_param.params_bezier[i][8] >> model_param.params_bezier[i][9]
	//				>> model_param.params_bezier[i][10];
	//		}
	//		infile.close();
	//	}
	//}

	//// （仮置き）pythonによる推定結果
	/*EstimateParameters(input_furi, input_kire, model_param);*/

	// タイムライン描画機能の初期化
	timeline = new Timeline();
	if ( motion )
		InitTimeline( timeline, *motion, deformation, timewarp_deformation, 0.0f );

	// 初期の視点を設定
	camera_yaw = -90.0f;
	camera_distance = 4.0f;

}


//
//  開始・リセット
//
void  MotionDeformationApp::Start()
{
	GLUTBaseApp::Start();
	
	on_animation = true;
	animation_time = 0.0f;
	frame_no = 0;
	Animation( 0.0f );
}


//
//  画面描画(たぶんEditの方が優先されてる)
//
void  MotionDeformationApp::Display()
{
	GLUTBaseApp::Display();

	// 動作変形後の姿勢を描画
	if ( deformed_posture )
	{
		glPushMatrix();

		glColor3f( 0.5f, 1.0f, 0.5f );
		DrawPosture( *deformed_posture );
		DrawPostureShadow( *deformed_posture, shadow_dir, shadow_color );

		glPopMatrix();
	}

	// 動作変形前の姿勢も描画（比較用）
	if ( draw_original_posture && org_posture )
	{
		glPushMatrix();

		if ( draw_postures_side_by_side )
			glTranslatef( -1.0f, 0.0f, 0.0f );

		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *org_posture );
		DrawPostureShadow( *org_posture, shadow_dir, shadow_color );

		glPopMatrix();
	}

	// 動作変形に使用するキー姿勢を描画（比較用）
	if ( draw_key_posture && draw_postures_side_by_side )
	{
		glPushMatrix();

		glTranslatef( 1.0f, 0.0f, 0.0f );

		glColor3f( 0.5f, 1.0f, 0.5f );
		DrawPosture( deformation.key_pose );
		DrawPostureShadow( deformation.key_pose, shadow_dir, shadow_color );

		glPopMatrix();
	}

	// ２つ目の動作の姿勢を描画
	//if (second_curr_posture)
	//{
	//	glPushMatrix();

	//	//steplong_Char00の腰の位置が違ったので強引に修正
	//	glTranslatef( 0.0f, 0.0f, 0.3f ); 

	//	glEnable(GL_BLEND);
	//	glColor4f(1.0f, 0.0f, 1.0f, 0.5f);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
	//	DrawPosture(*second_curr_posture);
	//	DrawPostureShadow(*second_curr_posture, shadow_dir, shadow_color);
	//	glDisable(GL_BLEND);

	//	glPopMatrix();
	//}

	// タイムラインを描画
	if ( timeline )
	{
		timeline->SetLineTime( 1, animation_time );
		timeline->DrawTimeline();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Deformation Base" );
	char  message[64];
	if ( motion )
	{
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
		DrawTextInformation( 1, message );
	}

	// 現在編集中のパラメータを表示
	char param_msg[128];
	const char* part_names[] = { "R_Foot", "L_Foot", "R_Hand", "L_Hand", "Hips", "Chest", "Head", "Kire" };

	// 選択中の項目と値を表示
	if (selected_param >= 0 && selected_param <= 7) 
	{
		float val = (selected_param == 7) ? kire : furi[selected_param];
		sprintf(param_msg, "EDIT: %s = %.2f (Use [ / ] to change)", part_names[selected_param], val);
		DrawTextInformation(4, param_msg); 
	}
}


//
//  ウィンドウサイズ変更
//
void  MotionDeformationApp::Reshape( int w, int h )
{
	GLUTBaseApp::Reshape( w, h );

	// タイムラインの描画領域の設定（画面の下部に配置）
	if ( timeline )
		timeline->SetViewAreaBottom( 0, 0, 0, 4, 32, 2 );
}


//
//  マウスクリック
//
void  MotionDeformationApp::MouseClick( int button, int state, int mx, int my )
{
	GLUTBaseApp::MouseClick( button, state, mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 入力動作トラック上のクリック位置に応じて、変形動作の再生時刻を変更
	if ( drag_mouse_l && ( selected_track_no == 0 ) )
	{
		animation_time = selected_time;
		Animation( 0.0f );
	}
}


//
//  マウスドラッグ
//
void  MotionDeformationApp::MouseDrag( int mx, int my )
{
	GLUTBaseApp::MouseDrag( mx, my );

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx );

	// 変形動作の再生時刻を変更
	if ( drag_mouse_l && ( selected_track_no == 0 ) )
	{
		animation_time = selected_time;
		drag_mouse_l = false;
		Animation( 0.0f );
		drag_mouse_l = true;
	}
}


//
//  キーボードのキー押下
//
void  MotionDeformationApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// 数字キーで入力動作・動作変形情報を変更
	//if ( ( key >= '1' ) && ( key <= '9' ) )
	//{
	//	InitMotion( key - '1' );
	//}

	// 数字キーで編集するレベルの選択（0-6が各部位のフリ・7がキレ）
	if (key >= '0' && key <= '6')
	{
		selected_param = key - '0';
	}
	else if (key == '7')
	{
		selected_param = 7;
	}

	//  レベルの増減 ( [ キーで減少、 ] キーで増加 )
	if (key == '[' || key == ']' || key == '{' || key == '}')
	{
		float delta = 0.0f; // 変化量を保持する変数

		// キーの種類に応じて変化量を決定する
		if (key == ']') delta = 1.0f;       // ] キーで +1.0
		else if (key == '[') delta = -1.0f;  // [ キーで -1.0
		else if (key == '}') delta = 0.1f;   // } (Shift + ]) で +0.1
		else if (key == '{') delta = -0.1f;  // { (Shift + [) で -0.1

		if (selected_param == 7) // キレレベル
		{
			kire += delta;
		}
		else if (selected_param >= 0 && selected_param < 7) // フリレベル
		{
			furi[selected_param] += delta;
		}

		// タイムラインの情報を更新
		//if (timeline && motion)
		//{
		//	TimeWarpingParam temp_param = timewarp_deformation;
		//	// タイムラインを現在の kire 値で再構築
		//	InitTimeline(timeline, *motion, deformation, temp_param, animation_time);
		//}
	}

	// d キーで表示姿勢の変更
	if ( key == 'd' )
	{
		if ( !draw_original_posture )
		{
			draw_original_posture = true;
			draw_postures_side_by_side = false;
		}
		else if ( draw_original_posture && !draw_postures_side_by_side )
		{
			//draw_key_posture = true;
			draw_postures_side_by_side = true;
		}
		else if ( draw_original_posture && draw_postures_side_by_side )
		{
			//draw_key_posture = false;
			draw_original_posture = false;
		}
	}

	// s キーでアニメーションの停止・再開
	if ( key == 's' )
		on_animation = !on_animation;

	// n キーで次のフレーム
	if ( ( key == 'n' ) && !on_animation && motion )
	{
		on_animation = true;
		Animation( motion->interval );
		on_animation = false;
	}

	// p キーで前のフレーム
	if ( ( key == 'p' ) && !on_animation && motion && ( frame_no > 0 ) )
	{
		on_animation = true;
		Animation( - motion->interval );
		on_animation = false;
	}

	// lキーで再生速度を上げる
	if (key == 'l')
		animation_speed += 0.1f;

	// jキーで再生速度を下げる
	if (key == 'j')
		animation_speed -= 0.1f;

	// kキーでキレ・フリの度合いの入力
	if (key == 'k') {
		// アニメーションを停止
		on_animation = false;

		if (selected_param == 7) { // キレの選択中
			// ポップアップ（コンソール注視）で入力を促す
			kire = ShowPopupInput("Kire Setting", "Enter new Kire value:", input_kire);
		}
		else if (selected_param >= 0 && selected_param < 7) { // 各部位のフリの選択中
			const char* names[] = { "R_Foot", "L_Foot", "R_Hand", "L_Hand", "Hips", "Chest", "Head" };
			furi[selected_param] = ShowPopupInput("Furi Setting", names[selected_param], input_furi);
		}

		std::cout << ">> Updated! Press 's' to resume." << std::endl;
	}

	// r キーでリセット
	if ( key == 'r' )
		Start();
	
	// o キーで変形後の動作を保存
	if ( key == 'o' )
	{
		// 元のファイル名から拡張子を除き、_deformed を付与する

		string output_file_name = current_file_name;
		// 最後の "." (ドット) の位置を探す
		size_t extension_pos = output_file_name.rfind('.');

		if (extension_pos != string::npos) {
			// 拡張子の直前に文字列を挿入 (例: "test.bvh" -> "test_deformed.bvh")
			output_file_name.insert(extension_pos, "_deformed");
		}
		else {
			// 拡張子がない場合 (例: "test" -> "test_deformed")
			output_file_name += "_deformed";
		}
		// 変形後の動作をBVH動作ファイルとして保存
		SaveDeformedMotionAsBVH( output_file_name.c_str() );

		std::ofstream outputfile("dataset.csv", std::ios::app);

		if (!outputfile.is_open()) {
			printf("Error: Could not open for writing.\n");
		}

		// 入力書き込み処理
		outputfile << kire << "," << furi[0] << ","
			<< model_param.right_foot_dist << ","
			<< model_param.left_foot_dist << ","
			<< model_param.right_hand_dist << ","
			<< model_param.left_hand_dist << ","
			<< model_param.head_dist << ","
			<< model_param.ChestVal << ","
			<< model_param.moving_ratio << ","
			<< model_param.interaction << ",";


		outputfile << kire << ",";
		for (int i = 0; i < 7; i++) {
			outputfile << furi[i] << (i == 6 ? "" : ",");
		}
		outputfile << "\n";
		outputfile.close();
	}
}


//
//  アニメーション処理
//
void  MotionDeformationApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// マウスドラッグ中はアニメーションを停止
	if ( drag_mouse_l )
		return;

	// 動作データが読み込まれていなければ終了
	if ( !motion )
		return;

	// 時間を進める
	animation_time += delta * animation_speed;
	if ( animation_time > motion->GetDuration() )
		animation_time -= motion->GetDuration();

	// 現在のフレーム番号を計算
	frame_no = animation_time / motion->interval;

	// 動作データから現在時刻の姿勢を取得
	motion->GetPosture( animation_time, *org_posture );

	// 動作変換（タイムワーピング）の情報の更新
	InitTimeDeformationParameter(animation_time, distanceinfo, timewarp_deformation, *motion, kire);

	// 動作変換（動作ワーピング）の情報の更新
	InitDeformationParameter(animation_time, distanceinfo, deformation, timewarp_deformation ,*motion, furi);

	// 動作変形（タイムワーピング）の適用後の姿勢の計算
	ApplyTimeWarping(animation_time, timewarp_deformation, *motion, before_frame_time, *deformed_posture);

	// 動作変形（動作ワーピング）の適用後の姿勢の計算
	weight = ApplyMotionDeformation( animation_time, deformation, *motion, *deformed_posture, timewarp_deformation, *deformed_posture );

	// 【追加】過去の動作からの累積オフセットを取得して適用
	Vector3f cumulative_pos;
	Quat4f cumulative_rot;

	// ここで高速化された関数を呼ぶ
	GetCumulativeOffset(animation_time, cumulative_pos, cumulative_rot);

	// 1. 回転の適用: 累積回転 * 現在の姿勢回転
	// Matrix3f同士の乗算やMatrix*Quatが直接できないため、Quatに変換して計算
	Quat4f current_rot_q;
	current_rot_q.set(deformed_posture->root_ori); // Matrix -> Quat

	Quat4f final_rot_q;
	// 累積回転を「左から」かけるか「右から」かけるかは、回転の定義（グローバル/ローカル）によりますが、
	// 通常、移動方向を変えるための全体の回転であれば左からかけます。
	final_rot_q.mul(cumulative_rot, current_rot_q);

	// 計算結果をMatrixに戻す
	deformed_posture->root_ori.set(final_rot_q); // Quat -> Matrix

	// 2. 位置の適用: (現在の位置) + (累積位置)
	// 回転の影響を受けた座標系であれば回転後に足すのが一般的です
	deformed_posture->root_pos = deformed_posture->root_pos + cumulative_pos;


	// 動作変形後のデータを取得
	ExportMotionData();

	// ２つ目の動作の姿勢を取得
 	second_motion->GetPosture(animation_time, *second_curr_posture);

	//std::cout << "org_Posture: root_pos.x = " << org_posture->root_pos.x << std::endl;
	//std::cout << "second_curr_Posture: root_pos.x = " << second_curr_posture->root_pos.x << std::endl;

}


//
// 動作データのCSV出力処理
//
void MotionDeformationApp::ExportMotionData()
{
	// 必要なデータが揃っていなければ何もしない
	if (!deformed_posture || !my_human_body || !csv_file.is_open())
		return;

	// 1. 順運動学計算 (Forward Kinematics)
	// 現在の変形後姿勢から、空間上の座標を計算
	static vector< Matrix4f > segment_frames;
	ForwardKinematics(*deformed_posture, segment_frames);

	float distances[NUM_PRIMARY_SEGMENTS];
	float total_dist = 0.0f;

	// 2. 各部位の移動距離計算
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
	{
		// HumanBodyを使って、その部位に対応する関節番号を取得
		int seg_no = my_human_body->GetPrimarySegment((PrimarySegmentType)i);

		// 有効な関節番号であれば計算
		if (seg_no != -1 && seg_no < segment_frames.size())
		{
			// Matrix4fから平行移動成分をVector3fで取り出す
			Vector3f vec_pos;
			segment_frames[seg_no].get(&vec_pos);

			// 距離計算のためにPoint3fへ変換
			Point3f current_pos = vec_pos;

			// 最初のフレーム付近は移動距離0とする（初期化）
			if (frame_no == 0 || animation_time <= 0.05f) {
				distances[i] = 0.0f;
			}
			else {
				// 前フレームとの距離を計算
				distances[i] = current_pos.distance(prev_segment_positions[i]);
			}

			// 次のフレームのために現在位置を保存
			prev_segment_positions[i] = current_pos;

			total_dist += distances[i];
		}
		else {
			distances[i] = 0.0f;
		}
	}

	// 3. CSVへの書き出し
	csv_file << animation_time << "," << frame_no;

	// 各部位のデータを出力
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) {
		csv_file << "," << distances[i];
	}
	// 合計値と改行を出力
	csv_file << "," << total_dist << std::endl;
}


//
//  入力動作の初期化
//
void  MotionDeformationApp::InitMotion( int no )
{
	// テストケース1
	if ( no == 0 )
	{
		// サンプルBVH動作データを読み込み
		//LoadBVH( "stepshort_new_Char00.bvh" ); //3
		//LoadBVH("radio_middle_3_Char00.bvh"); //4
		//LoadBVH("fight_punch.bvh"); //4
		//LoadBVH("radio_middle_2_Char00.bvh"); //4

		//LoadBVH("pointshort_Char00.bvh");
		LoadBVH("radio_new_3_Char00.bvh");
		LoadSecondBVH("radio_long_4_Char00.bvh");

		//LoadSecondBVH("steplong_Char00.bvh");
		//LoadSecondBVH("radio_long_3_Char00.bvh");
		//LoadSecondBVH("fight_punch_key.bvh");
		//LoadSecondBVH("radio_long_2_Char00.bvh"); //4
		if ( !motion )
			return;
	}

	// 以下、他のテストケースを追加する
	/*else if ( no == 1 )
	{
	}*/
}

//
//  動作変形情報の初期化
//
void  MotionDeformationApp::InitParameter()
{
	// タイムワーピング情報の初期化
	InitTimeDeformationParameter(distanceinfo, timewarp_deformation, *motion, kire);

	// 動作変形（動作ワーピング）情報の初期化（キー時刻＋キー姿勢の右手の目標位置を指定）
	InitDeformationParameter(*motion, 0.80f, 0.70f, 0.70f, 0, 15, Vector3f(0.0f, -0.2f, 0.0f), deformation);
}


//
//  動作変形情報にもとづくタイムラインの初期化
//
void  MotionDeformationApp::InitTimeline( Timeline * timeline, const Motion & motion, const MotionWarpingParam & motiondeform, TimeWarpingParam& timedeform, float curr_time )
{
	// タイムラインの時間範囲を設定
	timeline->SetTimeRange( -0.25f, motion.GetDuration() + 0.25f );

	// 全要素・縦棒の情報をクリア
	timeline->DeleteAllElements();
	timeline->DeleteAllLines();

	// 変形前の動作を表す要素を設定
	timeline->AddElement( 0.0f, motion.GetDuration(), Color4f( 1.0f, 1.0f, 1.0f, 1.0f ), motion.name.c_str(), 0 );

	/*
	// 動作変形の範囲を表す要素を設定
	timeline->AddElement( motiondeform.key_time - motiondeform.blend_in_duration, motiondeform.key_time + motiondeform.blend_out_duration, 
		Color4f( 0.5f, 1.0f, 0.5f, 1.0f ), "deformation", 1 );

	// 動作変形のキー時刻を表す縦線を設定
	timeline->AddLine( motiondeform.key_time, Color4f( 1.0f, 0.0f, 0.0f, 1.0f ) );
	*/

	// タイムワーピングの要素を設定
	// 計算用変数
	TimeWarpingParam beforetimedeform = { 1000000,1000000,1000000,1000000 };

	// タイムワーピングの情報を回す
	int i = 1;
	for (float warptime = 0; warptime <= motion.GetDuration(); warptime += motion.interval )
	{
		// タイムワーピングの情報を取得
		InitTimeDeformationParameter(warptime, distanceinfo, timedeform, motion, kire);
		// 前のフレームとタイムワーピングの情報が違うならタイムラインに設定
		if (beforetimedeform.warp_in_duration_time != timedeform.warp_in_duration_time)
		{
			//printf("warp_in%d:%f\nwarp_out%d:%f\nwarp_bef%d:%f\nwarp_aft%d:%f\n\n",i,timedeform.warp_in_duration_time,i,timedeform.warp_out_duration_time,i,timedeform.warp_key_time,i,timedeform.after_key_time);
			std::string str = "diff";
			str += std::to_string(i);
			const char* cstr = str.c_str();
			timeline->AddElement(timedeform.warp_in_duration_time, timedeform.warp_out_duration_time, Color4f(static_cast<float>(i) / 10.0f, 0.3f, 0.7f, 1.0f),cstr, i%3+1);
			timeline->AddLine(timedeform.warp_key_time, Color4f(1.0f, 0.0f, 0.0f, 1.0f));
			timeline->AddLine(timedeform.after_key_time, Color4f(0.0f, 0.0f, 1.0f, 1.0f));
			i++;
		}
		beforetimedeform = timedeform;
	}


	// 動作再生時刻を表す縦線を設定
	timeline->AddLine( curr_time, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ) );
}


//
//  BVH動作ファイルの読み込み、骨格・姿勢の初期化
//
void  MotionDeformationApp::LoadBVH( const char * file_name )
{
	// ファイル名を記憶
	current_file_name = file_name;

	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );

	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion )
		return;

	// 骨格・動作・姿勢の削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;
	if ( org_posture )
		delete  org_posture;
	if ( deformed_posture )
		delete  deformed_posture;
 
	// 動作変形に使用する動作・姿勢の初期化
	motion = new_motion;
	curr_posture = new Posture();
	InitPosture( *curr_posture, motion->body );
	org_posture = new Posture();
	InitPosture( *org_posture, motion->body );
	deformed_posture = new Posture();
	InitPosture( *deformed_posture, motion->body );
}


//
// ２つ目の動作ファイルの読み込み（上とほぼ同じ）
//
void  MotionDeformationApp::LoadSecondBVH(const char* file_name)
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion* new_motion = LoadAndCoustructBVHMotion(file_name);

	// BVHファイルの読み込みに失敗したら終了
	if (!new_motion)
		return;

	// 現在使用している骨格・動作・姿勢を削除
	if (second_motion && second_motion->body)
		delete  second_motion->body;
	if (second_motion)
		delete  second_motion;
	if (second_curr_posture)
		delete  second_curr_posture;

	// 動作再生に使用する動作・姿勢を初期化
	second_motion = new_motion;
	second_curr_posture = new Posture(second_motion->body);
}


//
// 累積オフセットの計算 
//

void MotionDeformationApp::GetCumulativeOffset(float current_time, Vector3f& out_pos_offset, Quat4f& out_rot_offset)
{
	// パラメータ変更の検知
	// スライダー操作などで「動作の大きさ(furi)」が変わった場合のみ、
	// 過去の軌道も再計算する必要があります。
	bool is_param_changed = false;
	for (int i = 0; i < 7; i++) {
		// 浮動小数の誤差を考慮して比較
		if (fabs(prev_furi[i] - furi[i]) > 0.001f) {
			is_param_changed = true;
			prev_furi[i] = furi[i]; // 現在の値を保存して更新
		}
	}

	// パラメータが変わった場合はキャッシュをクリアし、全区間を再計算させる
	if (is_param_changed) {
		cached_segment_pos_offsets.clear();
		cached_segment_rot_offsets.clear();
	}

	// オフセットの計算と蓄積
	// 出力用変数の初期化
	out_pos_offset.set(0.0f, 0.0f, 0.0f);       // 位置オフセットは(0,0,0)
	out_rot_offset.set(0.0f, 0.0f, 0.0f, 1.0f); // 回転オフセットは単位クォータニオン(回転なし)

	int segment_count = 0; // 処理済みの動作区間数をカウント

	// distanceinfo（全フレームの動き情報）を走査して、動作の区切りを見つける
	for (int i = 0; i < (int)distanceinfo.size() - 1; i++)
	{
		// 動作の開始点 (movecheck: 0 -> 1) を検出
		if (distanceinfo[i].movecheck == 0 && distanceinfo[i + 1].movecheck == 1)
		{
			// この動作区間の開始フレーム
			int start_frame = i + 1;

			// 区間終了の探索（停止期間も含めて「次の動作の開始」までを1区間とする）
			// まず、現在の動作が止まる地点 (1 -> 0) を探す
			int current_move_end = start_frame;
			for (int j = start_frame; j < (int)distanceinfo.size() - 1; j++) {
				if (distanceinfo[j].movecheck == 1 && distanceinfo[j + 1].movecheck == 0) {
					current_move_end = j;
					break;
				}
			}

			// 次に、そこから再び動作が始まる地点 (0 -> 1) を探す
			int next_start_frame = (int)distanceinfo.size(); // 見つからなければ最後まで
			bool found_next = false;
			for (int j = current_move_end; j < (int)distanceinfo.size() - 1; j++)
			{
				if (distanceinfo[j].movecheck == 0 && distanceinfo[j + 1].movecheck == 1)
				{
					next_start_frame = j + 1;
					found_next = true;
					break;
				}
			}

			// この区間の終了時刻（＝次の動作の開始時刻）
			float next_start_time = next_start_frame * motion->interval;

			// 累積への加算判定
			// 現在時刻が「次の動作の開始時刻」を過ぎている場合、
			// この区間（今の動作＋停止時間）は完了した「過去の動作」とみなせるため、オフセットを加算する。
			if (current_time >= next_start_time)
			{
				// キャッシュにこの区間のデータが既に存在するか確認
				if (segment_count < (int)cached_segment_pos_offsets.size())
				{
					// [高速化] 計算済みならキャッシュから値を足すだけ

					// 位置の累積
					out_pos_offset = out_pos_offset + cached_segment_pos_offsets[segment_count];

					// 回転の累積 (現在の累積回転 * 今回の回転オフセット)
					Quat4f temp = out_rot_offset;
					out_rot_offset.mul(temp, cached_segment_rot_offsets[segment_count]);
				}
				else
				{
					// [計算] キャッシュがない場合のみ、重いIK計算を行う

					// パラメータ計算のための基準時刻を設定
					// (区間内の確実に動いている時刻を指定することで、正しく動作パラメータを取得させる)
					float calc_time = start_frame * motion->interval;

					// その時刻における変形パラメータを取得 (IK計算などが走る)
					MotionWarpingParam seg_param;
					InitDeformationParameter(calc_time, distanceinfo, seg_param, timewarp_deformation, *motion, furi);

					// --- 位置の差分計算 (変形後 - 変形前) ---
					Vector3f pos_diff = seg_param.key_pose.root_pos - seg_param.org_pose.root_pos;

					// --- 回転の差分計算 (変形後 * 変形前の逆回転) ---
					// Matrix3f同士の演算ができないため、Quat4fに変換して計算
					Quat4f q_key, q_org;
					q_key.set(seg_param.key_pose.root_ori); // 行列 -> Quat
					q_org.set(seg_param.org_pose.root_ori); // 行列 -> Quat

					// 元の姿勢の逆回転（共役クォータニオン）を作成
					Quat4f q_org_inv;
					q_org_inv.x = -q_org.x;
					q_org_inv.y = -q_org.y;
					q_org_inv.z = -q_org.z;
					q_org_inv.w = q_org.w;

					// 差分回転 = KeyRot * OrgRot^-1
					Quat4f rot_diff;
					rot_diff.mul(q_key, q_org_inv);

					// --- 計算結果をキャッシュに保存 ---
					cached_segment_pos_offsets.push_back(pos_diff);
					cached_segment_rot_offsets.push_back(rot_diff);

					// --- 結果を累積に加算 ---
					out_pos_offset = out_pos_offset + pos_diff;

					Quat4f temp = out_rot_offset;
					out_rot_offset.mul(temp, rot_diff);
				}

				segment_count++; // 処理完了した区間数をインクリメント
			}
			else
			{
				// まだ次の動作が始まっていない（現在動作中、またはその後の停止中）なら、
				// この区間は「現在進行中」なので累積（過去分）には含めない。
				// (現在進行中の変形は ApplyMotionDeformation でリアルタイムに処理される)
				break;
			}

			// 次のループ探索位置を更新 (次の動作の開始地点の手前へ)
			// ループ末尾で i++ されるため -1 しておく
			i = next_start_frame - 1;
		}
	}
	// キャッシュの整理
	// 時間が巻き戻ったりリセットされた場合、未来のキャッシュが残っている可能性があるので削除する
	if (segment_count < (int)cached_segment_pos_offsets.size()) {
		cached_segment_pos_offsets.resize(segment_count);
		cached_segment_rot_offsets.resize(segment_count);
	}
}

//
//  変形後の動作をBVH動作ファイルとして保存
//

void  MotionDeformationApp::SaveDeformedMotionAsBVH( const char * file_name )
{
	// 変形後の動作データを生成
	Motion* deformed_motion = GenerateDeformedMotion(deformation, *motion, distanceinfo, kire, furi);
	if (!deformed_motion) return;

	// テンプレートとなるBVHファイルを読み込む
	BVH template_bvh( current_file_name.c_str() );

	if (!template_bvh.IsLoadSuccess()) {
		printf("Error: Template BVH (radio_long_3_Char00.bvh) load failed.\n");
		delete deformed_motion;
		return;
	}

	int num_frames = deformed_motion->num_frames;
	int num_channels = template_bvh.GetNumChannel();
	double* data = new double[num_frames * num_channels];

	// 全フレームループ
	for (int f = 0; f < num_frames; f++)
	{
		Posture& pose = deformed_motion->frames[f];

		for (int c = 0; c < num_channels; c++)
		{
			const BVH::Channel* channel = template_bvh.GetChannel(c);
			const BVH::Joint* joint = channel->joint;
			int joint_index = joint->index; // SimpleHumanのJoint indexと一致すると仮定

			double value = 0.0;

			// 位置情報の書き出し
			if (channel->type == BVH::X_POSITION ||
				channel->type == BVH::Y_POSITION ||
				channel->type == BVH::Z_POSITION)
			{
				if (joint->parent == NULL) {
					// ルート関節は Posture の root_pos を使用
					if (channel->type == BVH::X_POSITION) value = pose.root_pos.x / 0.01; // cm -> m 逆変換(bvh_scaleが0.01の場合)
					if (channel->type == BVH::Y_POSITION) value = pose.root_pos.y / 0.01;
					if (channel->type == BVH::Z_POSITION) value = pose.root_pos.z / 0.01;
				}
				else {
					// 子関節は OFFSET（骨の長さ）を使用することで潰れるのを防ぐ
					if (channel->type == BVH::X_POSITION) value = joint->offset[0];
					if (channel->type == BVH::Y_POSITION) value = joint->offset[1];
					if (channel->type == BVH::Z_POSITION) value = joint->offset[2];
				}
			}
			// 回転情報の書き出し
			else
			{
				// ここで関節に対応する回転行列を取得
				Matrix3f rot_mat;

				if (joint->parent == NULL) {
					// ルート関節の回転は pose.root_ori に格納されている
					rot_mat = pose.root_ori;
				}
				else {
					// 子関節のインデックス補正
					int simple_joint_index = joint->index - 1;

					if (simple_joint_index >= 0 && simple_joint_index < deformed_motion->body->num_joints) {
						rot_mat = pose.joint_rotations[simple_joint_index];
					}
					else {
						rot_mat.setIdentity();
					}
				}

				double rx, ry, rz;
				Quat4f q;
				q.set(rot_mat);

				// オイラー角に変換 (YXZ順)
				QuatToEulerYXZ(q, ry, rx, rz);

				if (channel->type == BVH::X_ROTATION) value = rx;
				if (channel->type == BVH::Y_ROTATION) value = ry;
				if (channel->type == BVH::Z_ROTATION) value = rz;
			}

			data[f * num_channels + c] = value;
		}
	}

	// 保存実行
	template_bvh.SetMotion(num_frames, deformed_motion->interval, data);
	template_bvh.Save(file_name);

	delete[] data;
	delete deformed_motion;
}


//
// 統計モデルを用いたパラメータの推定
//
void  MotionDeformationApp::EstimateParameters(float input_furi, float input_kire, ModelParam param)
{
	// キレの推定
	kire = param.params_kire[0] * input_kire
		+ param.params_kire[1] * input_furi
		+ param.params_kire[2] * param.right_foot_dist
		+ param.params_kire[3] * param.left_foot_dist
		+ param.params_kire[4] * param.right_hand_dist
		+ param.params_kire[5] * param.left_hand_dist
		+ param.params_kire[6] * param.head_dist
		+ param.params_kire[7] * param.ChestVal
		+ param.params_kire[8] * param.moving_ratio
		+ param.params_kire[9] * param.interaction
		+ param.params_kire[10];

	// フリの推定
	for (int i = 0; i < 7; i++) {
		furi[i] = param.params_furi[i][0] * input_kire
			+ param.params_furi[i][1] * input_furi
			+ param.params_furi[i][2] * param.right_foot_dist
			+ param.params_furi[i][3] * param.left_foot_dist
			+ param.params_furi[i][4] * param.right_hand_dist
			+ param.params_furi[i][5] * param.left_hand_dist
			+ param.params_furi[i][6] * param.head_dist
			+ param.params_furi[i][7] * param.ChestVal
			+ param.params_furi[i][8] * param.moving_ratio
			+ param.params_furi[i][9] * param.interaction
			+ param.params_furi[i][10];
	}

	// ベジェ制御点の推定
	float estimated_vals[4];

	for (int i = 0; i < 4; i++) {
		estimated_vals[i] = param.params_bezier[i][0] * input_kire
			+ param.params_bezier[i][1] * input_furi
			+ param.params_bezier[i][2] * param.right_foot_dist
			+ param.params_bezier[i][3] * param.left_foot_dist
			+ param.params_bezier[i][4] * param.right_hand_dist
			+ param.params_bezier[i][5] * param.left_hand_dist
			+ param.params_bezier[i][6] * param.head_dist
			+ param.params_bezier[i][7] * param.ChestVal
			+ param.params_bezier[i][8] * param.moving_ratio
			+ param.params_bezier[i][9] * param.interaction
			+ param.params_bezier[i][10]; // 切片
	}

	// 推定値をタイムワーピングパラメータに適用
	auto clamp = [](float v) { return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v); };

	timewarp_deformation.bezier_control1.x = clamp(estimated_vals[0]);
	timewarp_deformation.bezier_control1.y = clamp(estimated_vals[1]);
	timewarp_deformation.bezier_control2.x = clamp(estimated_vals[2]);
	timewarp_deformation.bezier_control2.y = clamp(estimated_vals[3]);
}

//
// 末端部位の移動距離の合計の情報の初期化
//
void InitDistanceParameter(vector<DistanceParam> & param)
{
	param.resize(0);
}


//
// 肩を利用したねじれの計算
//

float CalcChestVal(const Motion* motion)
{
	if (!motion) return 0.0;

	// 1. 関節の特定
	HumanBody human_body(motion->body);

	// 候補リストを使って存在するものを登録する
	const char* r_shoulder_candidates[] = { "RightArm", "RightShoulder", "RightCollar" };
	const char* l_shoulder_candidates[] = { "LeftArm",  "LeftShoulder",  "LeftCollar" };

	// 右肩の特定
	for (auto name : r_shoulder_candidates) {
		human_body.SetPrimaryJoint(JOI_R_SHOULDER, name);
		if (human_body.GetPrimaryJoint(JOI_R_SHOULDER) != -1) break;
	}

	// 左肩の特定
	for (auto name : l_shoulder_candidates) {
		human_body.SetPrimaryJoint(JOI_L_SHOULDER, name);
		if (human_body.GetPrimaryJoint(JOI_L_SHOULDER) != -1) break;
	}

	int rShldrIdx = human_body.GetPrimaryJoint(JOI_R_SHOULDER);
	int lShldrIdx = human_body.GetPrimaryJoint(JOI_L_SHOULDER);

	std::vector<float> torsionValues;
	torsionValues.reserve(motion->num_frames);

	// 計算用の一時変数
	Posture temp_posture;
	std::vector< Matrix4f > seg_frames;
	std::vector< Point3f > joint_positions;

	// 2. 全フレームを走査
	for (int i = 0; i < motion->num_frames; ++i)
	{
		// 時間から姿勢を取得
		float time = i * motion->interval;
		motion->GetPosture(time, temp_posture);

		// 順運動学計算 (FK) で、関節のグローバル座標を取得
		ForwardKinematics(temp_posture, seg_frames, joint_positions);

		// 右肩と左肩の位置取得
		Point3f rPos = joint_positions[rShldrIdx];
		Point3f lPos = joint_positions[lShldrIdx];

		// Z軸（奥行き）の差分を計算
		// パンチ動作等は、片方の肩が前、もう片方が後ろに行くため、この値が大きく変動します
		float zDiff = rPos.z - lPos.z;

		torsionValues.push_back(zDiff);
	}

	// 3. 分散を計算
	if (torsionValues.empty()) return 0.0;

	// 平均値
	float sum = std::accumulate(torsionValues.begin(), torsionValues.end(), 0.0);
	float mean = sum / torsionValues.size();

	// 分散
	float sqSum = 0.0;
	for (const float val : torsionValues) {
		sqSum += std::pow(val - mean, 2);
	}
	float variance = sqSum / torsionValues.size();

	return variance;
}


//
// Windowsの標準機能を使って、入力ダイアログを表示する関数
//
float ShowPopupInput(const char* title, const char* prompt, float current_val) 
{
	char buffer[128];
	sprintf(buffer, "%f", current_val); // 現在の値を初期値としてセット

	// コンソール入力をポップアップ風に見せる

	std::cout << "--- " << title << " ---" << std::endl;
	std::cout << prompt << " (Current: " << current_val << ")" << std::endl;
	std::cout << ">> Please enter value in the console and press Enter." << std::endl;

	// ウィンドウを前面に持ってくる
#ifdef _WIN32
	SetForegroundWindow(GetConsoleWindow());
#endif

	float val;
	std::cin >> val;
	return val;
}


//
// 末端部位の移動距離を測定
//
void CheckDistance(const Motion& motion, vector<DistanceParam> & param, ModelParam& m_param, const char ** segment_names)
{
	// 骨格の追加情報を生成
	// キャラクタの骨格情報
	HumanBody* human_body;

	human_body = new HumanBody(motion.body);
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
	{
		human_body->SetPrimarySegment((PrimarySegmentType)i, segment_names[i]);
	}
		
	// 計算用変数
	Point3f before_segment_positions[NUM_PRIMARY_SEGMENTS] ; // 前フレームの主要体節の位置

	// 末端部位の位置を取得
	// 動作終了フレームまで繰り返す
	for (float animation_time = 0; animation_time < motion.GetDuration(); animation_time += motion.interval)
	{
		// 計算用変数
		Posture * curr_posture = & motion.frames[0]; // 最初の姿勢で初期化

		// 動作データから現在時刻の姿勢を取得
		motion.GetPosture(animation_time, *curr_posture);

		// 順運動学計算により主要体節の位置を計算・記録
		static vector< Matrix4f > segment_frames;
		Vector3f vec;
		Point3f segment_positions[NUM_PRIMARY_SEGMENTS];

		ForwardKinematics(*curr_posture, segment_frames);
		for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) 
		{
			int seg_no = human_body->GetPrimarySegment((PrimarySegmentType)i);
			if (seg_no != -1)
			{
				segment_frames[seg_no].get(&vec);
				segment_positions[i] = vec;
			}
		}

		// もし初めのフレームなら前フレームの主要体節の位置を現在の位置と同一に設定
		if (animation_time == 0)
		{
			for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
				before_segment_positions[i] = segment_positions[i];
		}

		// 両手足のフレーム間の距離を計算
		float right_ankle_dist = segment_positions[0].distance(before_segment_positions[0]);
		float left_ankle_dist = segment_positions[1].distance(before_segment_positions[1]);
		float right_hand_dist = segment_positions[2].distance(before_segment_positions[2]);
		float left_hand_dist = segment_positions[3].distance(before_segment_positions[3]);

		float head_dist = segment_positions[6].distance(before_segment_positions[6]);

		// 末端部位の移動距離の合計を計算
		float total_dist = right_ankle_dist + left_ankle_dist + right_hand_dist + left_hand_dist;

		// 前フレームの両手足の位置を更新
		for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
		{
			int  seg_no = human_body->GetPrimarySegment((PrimarySegmentType)i);
			if (seg_no != -1)
			{
				before_segment_positions[i] = segment_positions[i];
			}
		}

		m_param.right_foot_dist += right_ankle_dist;
		m_param.left_foot_dist += left_ankle_dist;
		m_param.right_hand_dist += right_hand_dist;
		m_param.left_hand_dist += left_hand_dist;
		m_param.head_dist += head_dist;

		// 保存用変数
		DistanceParam d;

		// 末端部位の移動距離の合計を格納
		d.distanceadd=total_dist;

		// 動作が動いているかどうかの初期化
		d.movecheck = 1;

		// 動作が開始したかどうかの判断の初期化
		d.move_start = true;

		// 動作が動いているかどうかの閾値の初期化
		d.move_amount = 10000;

		// 現フレームの情報を格納する
		param.push_back(d);
	}

	// 統計モデルの情報を更新
	m_param.right_foot_dist = m_param.right_foot_dist / motion.num_frames;
	m_param.left_foot_dist = m_param.left_foot_dist / motion.num_frames;
	m_param.right_hand_dist = m_param.right_hand_dist / motion.num_frames;
	m_param.left_hand_dist = m_param.left_hand_dist / motion.num_frames;
	m_param.head_dist = m_param.head_dist / motion.num_frames;

	// 平滑化(前後10フレーム)
	for (int i = 10; i < param.size() - 10; i++)
	{
		for (int j = 1; j < 11; j++)
		{
			param[i].distanceadd += param[i + j].distanceadd;
			param[i].distanceadd += param[i - j].distanceadd;
		}

		param[i].distanceadd = param[i].distanceadd / 21;
	}

	// --- 後半：セグメンテーションロジックの刷新 ---

	// 1. 統計情報の取得（全体の平均・最小・最大）
	// 単純なmin/avgではなく、ソートしてパーセンタイル（分位点）を取得する
	std::vector<float> sorted_dists;
	sorted_dists.reserve(param.size());
	float avg_val = 0.0f;

	for (const auto& p : param) {
		sorted_dists.push_back(p.distanceadd);
		avg_val += p.distanceadd;
	}
	avg_val /= param.size();

	// 小さい順に並べ替え
	std::sort(sorted_dists.begin(), sorted_dists.end());

	// ノイズ（0などの外れ値）を除外するため、下位10%〜20%の値をベースラインとする
	int baseline_index = (int)(sorted_dists.size() * 0.20f); // 下位20%を採用
	if (baseline_index >= sorted_dists.size()) baseline_index = sorted_dists.size() - 1;
	float robust_min_val = sorted_dists[baseline_index];

	// 2. 閾値の設定 (ヒステリシス)
	// ベースライン（実質最小値）と平均値の間を取る
	// ノイズフロアよりも確実に高い位置に閾値を設定する
	float base_threshold = robust_min_val + (avg_val - robust_min_val) * 0.5f;

	// デバッグ用出力（コンソールで確認したい場合）
	// std::cout << "Avg: " << avg_val << " Min(raw): " << sorted_dists[0] 
	//           << " RobustMin(15%): " << robust_min_val << " Thresh: " << base_threshold << std::endl;

	float high_thresh = base_threshold * 1.2f; // 開始判定用
	float low_thresh = base_threshold * 0.8f;  // 終了判定用

	// 3. 閾値判定 (Pass 1)
	bool is_moving = false;
	for (int i = 0; i < param.size(); i++) {
		if (!is_moving) {
			// 静止 -> 動作：高い閾値を超える必要がある
			if (param[i].distanceadd > high_thresh) is_moving = true;
		}
		else {
			// 動作 -> 静止：低い閾値を下回る必要がある
			if (param[i].distanceadd < low_thresh) is_moving = false;
		}
		param[i].movecheck = is_moving ? 1 : 0;
		param[i].move_amount = base_threshold; // デバッグ表示用に保存
	}

	// 4. ギャップ結合 (Gap Closing) (Pass 2)
	// 動作と動作の間の隙間が短すぎる場合、繋げて一つの動作にする
	// これにより「不必要に細かく分割される」のを防ぐ
	int min_gap_frames = 20; // 例: 20フレーム(約0.6秒)未満の隙間は埋める
	int last_move_end = -1;

	// 最初の動作開始点を探す
	int first_move_idx = -1;
	for (int i = 0; i < param.size(); i++) {
		if (param[i].movecheck == 1) {
			first_move_idx = i;
			break;
		}
	}

	if (first_move_idx != -1) {
		last_move_end = first_move_idx; // 仮置き
		// 動作終了点を探して更新していく
		for (int i = first_move_idx; i < param.size(); i++) {
			if (param[i].movecheck == 1) {
				// 前回の動作終了から今回の動作開始までの距離をチェック
				int gap = i - last_move_end;
				// ギャップがあり、かつ指定フレーム未満なら埋める
				if (gap > 1 && gap < min_gap_frames) {
					for (int k = last_move_end + 1; k < i; k++) {
						param[k].movecheck = 1;
					}
				}
				last_move_end = i;
			}
		}
	}

	// 5. 短小ノイズ除去 (Noise Removal) (Pass 3)
	// 動作区間が短すぎる場合、ノイズとみなして静止にする
	// これにより「一瞬だけの誤検出」を防ぐ
	int min_duration_frames = 15; // 例: 15フレーム(0.5秒)未満の動作は無視
	int current_run = 0;
	for (int i = 0; i < param.size(); i++) {
		if (param[i].movecheck == 1) {
			current_run++;
		}
		else {
			if (current_run > 0 && current_run < min_duration_frames) {
				// 短すぎる区間を消去
				for (int k = i - current_run; k < i; k++) {
					param[k].movecheck = 0;
				}
			}
			current_run = 0;
		}
	}
	// 配列末尾の処理
	if (current_run > 0 && current_run < min_duration_frames) {
		for (int k = param.size() - current_run; k < param.size(); k++) {
			param[k].movecheck = 0;
		}
	}

	// 6. move_startフラグの設定（既存ロジックを踏襲）
	// 「一度でも動作が開始されたら、それ以降はずっとtrue」
	for (int i = 0; i < param.size(); i++)
	{
		if (param[i].movecheck == 1)
			param[i].move_start = true;
		else
		{
			param[i].move_start = false;
			if (i > 0) {
				if (param[i - 1].move_start)
					param[i].move_start = true;
			}
		}
	}

	// 7.動作密度の計算
	int move_count = 0;
	for (const auto& p : param)
	{
		if (p.movecheck == 1)
			move_count++;
	}
	if (!param.empty()) {
		m_param.moving_ratio = (float)move_count / param.size();
	}
	else
	{
		m_param.moving_ratio = 0.0f;
	}
}


//
//  動作変形情報にもとづく動作変形処理（タイムワーピング）
//

//
// 動作変形（タイムワーピング）の情報の初期化・更新
//
void  InitTimeDeformationParameter(
	vector<DistanceParam>& distance, TimeWarpingParam& param, Motion motion, float kire)
{
	InitTimeDeformationParameter(0.0f, distance, param, motion, kire);
}

//
//  動作変形（タイムワーピング）の情報の初期化・更新
//
void  InitTimeDeformationParameter(
	float now_time, vector<DistanceParam>& distance, TimeWarpingParam& param, Motion motion, float kire)
{
	// 現在時刻のフレームを計算
	int now_frame = now_time / motion.interval;

	// 動作が始まっているならワーピングを開始
	if (distance[now_frame].move_start)
	{
		// 普通の動作のとき
		if (now_frame > 0 && now_frame < distance.size())
		{
			// 今の動作の始まりの時間がワーピング開始フレーム
			bool found_in = false;
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.warp_in_duration_time = i * motion.interval;
					found_in = true;
					break;
				}
			}
			// 見つからず、かつ0フレーム目が動作中なら0を開始点とする
			if (!found_in && distance[0].movecheck == 1) {
				param.warp_in_duration_time = 0.0f;
			}

			// 今の動作の終わりの時間がワーピング終了フレーム
			param.warp_out_duration_time = NULL;
			for (int i = now_frame; i < distance.size()-1; i++)
			{
				if (distance[i].movecheck == 0 && distance[i + 1].movecheck == 1)
				{
					param.warp_out_duration_time = i * motion.interval;
					break;
				}
				param.warp_out_duration_time = motion.GetDuration();
			}

			// 動いている時間の終わりがワーピング前のキー時刻
			bool found_key = false;
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.warp_key_time = i * motion.interval;
						found_key = true;
						break;
					}
				}
			}
			else
			{
				for (int i = now_frame; i < distance.size()-1; i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.warp_key_time = i * motion.interval;
						found_key = true;
						break;
					}
				}
			}
			// 最後まで動作している場合
			if (!found_key) {
				param.warp_key_time = motion.GetDuration();
			}
		}
		// 動作のはじめのフレーム
		else if (now_frame == 0)
		{
			// ワーピング開始フレームは0
			param.warp_in_duration_time = 0.0f;

			// 今の動作の終わりの時間がワーピング終了フレーム
			for (int i = now_frame; i < distance.size()-1; i++)
			{
				if (distance[i].movecheck == 0 && distance[i + 1].movecheck == 1)
				{
					param.warp_out_duration_time = i * motion.interval;
					break;
				}
			}

			// 動いている時間の終わりがワーピング前のキー時刻
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.warp_key_time = i * motion.interval;
						break;
					}
				}
			}
			else
			{
				for (int i = now_frame; i < distance.size()-1; i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.warp_key_time = i * motion.interval;
						break;
					}
				}
			}
		}
		// 動作の最終フレーム
		else
		{
			// 今の動作の始まりの時間がワーピング開始フレーム
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.warp_in_duration_time = i * motion.interval;
					break;
				}
			}

			// ワーピング終了フレームは動作の終了フレーム
			param.warp_out_duration_time = distance.size() * motion.interval;

			// 動いている時間の終わりがワーピング前のキー時刻
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.warp_key_time = i * motion.interval;
						break;
					}
				}
			}
			else
			{
				for (int i = now_frame; i < distance.size(); i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.warp_key_time = i * motion.interval;
						break;
					}
				}
			}
		}

		// ワーピング前のキー時刻の正規化時間を計算
		float warp_key_native_time = 
			(param.warp_key_time - param.warp_in_duration_time) / (param.warp_out_duration_time - param.warp_in_duration_time);

		// ワーピング後のキー時刻の正規化時間を計算
		float after_key_native_time = warp_key_native_time * kire;
		if (after_key_native_time >= 1)
			after_key_native_time = 1.0f;

		// ワーピング後のキー時刻を計算
		param.after_key_time =
			param.warp_in_duration_time + (param.warp_out_duration_time - param.warp_in_duration_time) * after_key_native_time;
	}
	// 動作が始まっていないときはワーピングを開始しない
	else if (distance[now_frame].move_start == false)
	{
		param.warp_in_duration_time = -2000.f;
		param.warp_out_duration_time = -10000.0f;
		param.warp_key_time = -1000.0;
		param.after_key_time = -1000.0;
	}
}


//
//  動作変形（タイムワーピング）の適用後の動作を生成
//

Motion* GenerateTimeDeformedMotion(TimeWarpingParam& deform, const DistanceParam &distinfo ,const Motion &motion,const float kire)
{
	Motion* deformed = NULL;
	Posture* deformed_posture = NULL;
	float before_frame_time = NULL;

	// 動作変形前の動作を生成
	deformed = new Motion(motion);
	deformed_posture = new Posture();

	// 各フレームの姿勢を変形
	for (int i = 0; i < motion.num_frames; i++)
	{
		//InitTimeDeformationParameter(motion.interval * i, distinfo, deform, motion, kire);
		ApplyTimeWarping(motion.interval * i, deform, motion, before_frame_time, *deformed_posture);
	}

	// 動作変形後の動作を返す
	return  deformed;
}


//
// タイムワーピングの実行
//

void ApplyTimeWarping(float now_time, TimeWarpingParam& deform, const Motion& input_motion, float& before_frame_time, Posture& output_pose)
{
	// もし現在時刻にタイムワーピングを適用するなら
	if (now_time > deform.warp_in_duration_time && now_time < deform.warp_out_duration_time)
	{
		// タイムワーピング実行後の姿勢
		Posture after_pose ;

		// タイムワーピング実行後の時刻を取得
		float warping_time = Warping(now_time, deform);

		// タイムワーピング実行後の姿勢を生成
		input_motion.GetPosture(warping_time, after_pose);

		// 前のフレームの姿勢と姿勢補間
		// 動作を滑らかにつなぐ努力
		Posture before_frame_pose;
		input_motion.GetPosture(before_frame_time, before_frame_pose);

		PostureInterpolation(after_pose, before_frame_pose, 0.5, output_pose);

		before_frame_time = warping_time;
	}
	else
	{
		// 姿勢を生成
		input_motion.GetPosture(now_time, output_pose);
		before_frame_time = now_time;
	}
}

//
// タイムワーピング実行後の時刻を取得
//

float Warping(float now_time, TimeWarpingParam& deform)
{
	// タイムワーピング前後のキー時刻の正規化時刻を計算する
	if (deform.warp_key_time == deform.after_key_time)
		deform.warp_key_time -= 0.033f; //intervalの値

	float warping_native_time; // タイムワーピング実行後の正規化時刻

	float in_time, out_time; // ベジェ曲線の開始時刻・終了時刻

	// ベジェ曲線の区間決定
	if (now_time <= deform.after_key_time)
	{
		in_time = deform.warp_in_duration_time;
		out_time = deform.after_key_time;
	}
	else
	{
		in_time = deform.after_key_time;
		out_time = deform.warp_out_duration_time;
	}

	// 現在時刻の正規化時刻を計算
	float now_native_time = (now_time - in_time) / (out_time - in_time);

	// ベジェ曲線の開始点・終了点の設定
	Point2f in, out;
	in.x = 0.0f; in.y = 0.0f;
	out.x = 1.0f; out.y = 1.0f;

	// ベジェ曲線の制御点
	Point2f half1,half2;

	// 制御点の座標を求める
	half1.x = 0.05f;
	half2.x = 0.95f;

	half1.y = 0.0f;
	half2.y = 1.0f;

	// 制御点の座標を求める
	/*half1 = deform.bezier_control1;
	half2 = deform.bezier_control2;*/

	// ベジェ曲線のパラメータtを求める(二分探索による近似値)
	float lower = 0.0f;
	float upper = 1.0f;
	float tolerance = 0.000001;
	float t = now_native_time; // 初期値

	for (int i = 0; i < 10; ++i)
	{
		Point2f point_at_t;
		CalcBezier(in, out, half1, half2, t, point_at_t);
		float x_at_t = point_at_t.x;

		if (std::abs(x_at_t - now_native_time) < tolerance)
			break;

		if(x_at_t < now_native_time)
		{
			lower = t;
		}
		else 
		{
			upper = t;
		}
		t = (upper + lower) / 2.0f;
	}

	// 結果保存用変数
	Point2f result;
	CalcBezier(in, out, half1, half2, t, result);

	warping_native_time = result.y;

	float before_in_time, before_out_time; // タイムワーピング実行前の開始時刻・終了時刻

	// ベジェ曲線の区間決定
	if (now_time <= deform.after_key_time)
	{
		before_in_time = deform.warp_in_duration_time;
		before_out_time = deform.warp_key_time;
	}
	else
	{
		before_in_time = deform.warp_key_time;
		before_out_time = deform.warp_out_duration_time;
	}

	// タイムワーピング実行後の時刻を計算して返す	
	float warping_time = before_in_time + (before_out_time - before_in_time) * warping_native_time;

	//std::ofstream outputfile("warpingtime_output.csv", std::ios::app);
	//outputfile << warping_time;
	//outputfile << ',';
	//outputfile << warping_native_time;
	//outputfile << '\n';
	//outputfile.close();

	return warping_time;
}

//
//  ベジェ曲線上の点を計算
//
void CalcBezier(Point2f in, Point2f out, Point2f half1, Point2f half2, float t, Point2f& result)
{
	float u = 1.0 - t;
	float tt = t * t;
	float uu = u * u;
	float uuu = uu * u;
	float ttt = tt * t;

	result.x = uuu * in.x + 3 * uu * t * half1.x + 3 * u * tt * half2.x + ttt * out.x;
	result.y = uuu * in.y + 3 * uu * t * half1.y + 3 * u * tt * half2.y + ttt * out.y;
}

//
//  動作変形情報にもとづく動作変形処理（モーションワーピング）
//


//
//  動作変形（動作ワーピング）の情報の初期化・更新
//
void  InitDeformationParameter( 
	const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	MotionWarpingParam & param )
{
	param.key_time = key_time;
	param.blend_in_duration = blend_in_duration;
	param.blend_out_duration = blend_out_duration;
	motion.GetPosture( param.key_time, param.org_pose );
	param.key_pose = param.org_pose;
}


//
//  動作変形（動作ワーピング）の情報の初期化・更新
//
void  InitDeformationParameter( 
	const Motion & motion, float key_time, float blend_in_duration, float blend_out_duration, 
	int base_joint_no, int ee_joint_no, Point3f ee_joint_translation, 
	MotionWarpingParam & param )
{
	InitDeformationParameter( motion, key_time, blend_in_duration, blend_out_duration, param );

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	vector< Point3f >  joint_position_frame_array;
	ForwardKinematics( param.key_pose, seg_frame_array, joint_position_frame_array );

	// 指定関節の目標位置
	Point3f  ee_pos;
	ee_pos = joint_position_frame_array[ ee_joint_no ];
	ee_pos.add( ee_joint_translation );

	// キー姿勢の指定部位の位置を移動
	ApplyInverseKinematicsCCD( param.key_pose, base_joint_no, ee_joint_no, ee_pos );
}

//
//  動作変形（動作ワーピング）の情報の初期化・更新
//
void  InitDeformationParameter(
	float now_time, vector<DistanceParam> distance, MotionWarpingParam& param, TimeWarpingParam time_param, Motion& motion, float furi[])
{
	// 現在時刻のフレーム
	int now_frame = 0.00;

	// もし現在時刻にタイムワーピングを適用するなら
	if (now_time > time_param.warp_in_duration_time && now_time < time_param.warp_out_duration_time) {
		// タイムワーピング後の現在時刻を計算
		float warping_time = Warping(now_time, time_param);
		// 現在時刻のフレームを計算
		now_frame = warping_time / motion.interval;
	}
	else{
		now_frame = now_time / motion.interval;
	}

	// 応急処置
	if (now_frame >= motion.num_frames)
		now_frame = motion.num_frames -1;


	// 動作が始まっているならワーピングを開始
	if (distance[now_frame].move_start)
	{
		// 普通のフレーム
		if (now_frame > 0 && now_frame < distance.size())
		{
			// 今の動作の始まりの時間がワーピング開始フレーム
			bool found_in = false;
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.blend_in_duration = i * motion.interval;
					found_in = true;
					break;
				}
			}
			// 見つからず、かつ0フレーム目が動作中なら0を開始点とする
			if (!found_in && distance[0].movecheck == 1) {
				param.blend_in_duration = 0.0f;
			}

			// 今の動作の終わりの時間がワーピング終了フレーム
			for (int i = now_frame; i < distance.size() - 1; i++)
			{
				if (distance[i].movecheck == 0 && distance[i + 1].movecheck == 1)
				{
					param.blend_out_duration = i * motion.interval;
					break;
				}
				param.blend_out_duration = motion.GetDuration();
			}

			// 動いている時間の終わりがキー時刻
			bool found_key = false;
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.key_time = i * motion.interval;
						found_key = true;
						break;
					}
				}
			}
			else if(distance[now_frame].movecheck == 1)
			{
				for (int i = now_frame; i < distance.size()-1; i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.key_time = i * motion.interval;
						found_key = true;
						break;
					}
				}
			}
			// 見つからなかった場合（最後まで動作している場合）、終了時刻を最後にする
			if (!found_key) 
			{
				param.key_time = motion.GetDuration();
			}
		}
		// 最初のフレーム
		else if (now_frame == 0)
		{
			// ワーピング開始フレームは0
			param.blend_in_duration = 0.0f;

			// 今の動作の終わりの時間がワーピング終了フレーム
			for (int i = now_frame; i < distance.size()-1; i++)
			{
				if (distance[i].movecheck == 0 && distance[i + 1].movecheck == 1)
				{
					param.blend_out_duration = i * motion.interval;
					break;
				}
			}

			// 動いている時間の終わりがキー時刻
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.key_time = i * motion.interval;
						break;
					}
				}
			}
			else
			{
				for (int i = now_frame; i < distance.size(); i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.key_time = i * motion.interval;
						break;
					}
				}
			}
		}
		// 最後のフレーム
		else
		{
			// 今の動作の始まりの時間がワーピング開始フレーム
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.blend_in_duration = i * motion.interval;
					break;
				}
			}

			// ワーピング終了フレームは動作の終了フレーム
			param.blend_out_duration = distance.size() * motion.interval;

			// 動いている時間の終わりがキー時刻
			if (distance[now_frame].movecheck == 0)
			{
				for (int i = now_frame; i > 0; i--)
				{
					if (distance[i].movecheck == 1)
					{
						param.key_time = i * motion.interval;
						break;
					}
				}
			}
			else
			{
				for (int i = now_frame; i < distance.size(); i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.key_time = i * motion.interval;
						break;
					}
				}
			}
		}

		// ワーピング前のキー時刻の姿勢を取得
		motion.GetPosture(param.key_time, param.org_pose);

		// ワーピング後のキー時刻の姿勢を取得するための計算

		// モーションワーピング後のキー姿勢を末端部位の位置変更により更新
		//UpdateKeyposeByPosition(param, motion, furi);

		// モーションワーピング後のキー姿勢を関節角度の回転速度の変更により更新
		UpdateKeyposeByVelocity(param, motion, furi);
	}


	// 動作が始まっていないときはワーピングを開始しない
	else if (distance[now_frame].move_start == false)
	{
		param.blend_in_duration = -1000.0f;
		param.blend_out_duration = 0;
		param.key_time = 0;
		motion.GetPosture(now_frame, param.org_pose);
		param.key_pose = param.org_pose;
	}
}

//
//　モーションワーピング後のキー姿勢を末端部位の位置変更により更新
//

void UpdateKeyposeByPosition(MotionWarpingParam& param, Motion& motion, float furi[])
{
	// 順運動学計算
	static vector< Matrix4f >  seg_frame_array;
	vector< Point3f >  joint_position_frame_array;
	ForwardKinematics(param.org_pose, seg_frame_array, joint_position_frame_array);
	param.key_pose = param.org_pose;

	// 1フレーム前の姿勢情報を取得(値は仮のもの)
	Posture before_posture = param.org_pose;

	// 取得するフレームの時間
	float before_time = param.key_time - motion.interval;
	if (before_time < 0.0f)
		before_time = 0.0f;

	// 1フレーム前の姿勢情報を取得
	motion.GetPosture(before_time, before_posture);

	// 数フレーム前の姿勢の順運動学計算
	static vector< Matrix4f >  before_seg_frame_array;
	vector< Point3f >  before_joint_position_frame_array;

	ForwardKinematics(before_posture, before_seg_frame_array, before_joint_position_frame_array);

	// 骨格の追加情報を生成
	// キャラクタの骨格情報
	HumanBody* human_body;

	human_body = new HumanBody(motion.body);

	const char* primary_segment_names[NUM_PRIMARY_SEGMENTS];

	GetAdaptiveSegmentNames(motion.body, primary_segment_names);

	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
	{
		human_body->SetPrimarySegment((PrimarySegmentType)i, primary_segment_names[i]);
	}

	// 末端部位ごとにモーションワーピング後のキー時刻の姿勢を変形する
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++)
	{
		// 末端部位の位置を取得
		Point3f segment_positions[NUM_PRIMARY_SEGMENTS];
		Point3f before_segment_positions[NUM_PRIMARY_SEGMENTS];
		Vector3f vec;
		Vector3f before_vec;

		int seg_no = human_body->GetPrimarySegment((PrimarySegmentType)i);
		if (seg_no != -1)
		{
			seg_frame_array[seg_no].get(&vec);
			segment_positions[i] = vec;
			before_seg_frame_array[seg_no].get(&before_vec);
			before_segment_positions[i] = before_vec;
		}

		// 末端部位の位置から末端部位の移動の向きを計算
		Vector3f move_vec;
		move_vec = segment_positions[i] - before_segment_positions[i];

		//// 軸足でない場合ノイズを付与

		// フリレベルを倍率として移動距離を設定
		move_vec = move_vec * furi[i];

		//目標となる新たな位置を算出
		Point3f ee_pos;
		ee_pos = before_segment_positions[i];
		ee_pos += move_vec;

		if (i == SEG_PELVIS)
		{
			// 腰（ルート）の場合は、IKを使わずに直接ルート座標を更新する
			param.key_pose.root_pos = ee_pos;
		}
		else
		{
			ApplyInverseKinematicsCCD(param.key_pose, -1, seg_no, ee_pos);
		}
	}
	delete human_body;
}

//
//　モーションワーピング後のキー姿勢を関節角度の回転速度の変更により更新
//
void UpdateKeyposeByVelocity(MotionWarpingParam& param, Motion& motion, float furi[])
{
	param.key_pose = param.org_pose;
	// 1フレーム前の姿勢情報を取得(値は仮のもの)
	Posture before_posture = param.org_pose;

	// 取得するフレームの時間
	float before_time = param.key_time - motion.interval;
	if (before_time < 0.0f)
		before_time = 0.0f;

	// 1フレーム前の姿勢情報を取得
	motion.GetPosture(before_time, before_posture);

	// 骨格の追加情報を生成
	// キャラクタの骨格情報
	HumanBody* human_body = new HumanBody(motion.body);

	const char* primary_joint_names[NUM_PRIMARY_JOINTS];

	// 自動判定関数を使用
	GetAdaptiveJointNames(motion.body, primary_joint_names);

	// 設定された名前を登録
	for (int i = 0; i < NUM_PRIMARY_JOINTS; i++)
	{
		human_body->SetPrimaryJoint((PrimaryJointType)i, primary_joint_names[i]);
	}

	for (int i = 0; i < NUM_PRIMARY_JOINTS; i++)
	{
		human_body->SetPrimaryJoint((PrimaryJointType)i, primary_joint_names[i]);
	}

	// クォータニオンを用いた関節角度の回転速度の変更
	Quat4f org_q, before_q, before_inv_q, diff_q;
	Quat4f identity_q(0.0f, 0.0f, 0.0f, 1.0f); // 単位クォータニオン（回転ゼロ）
	Quat4f scaled_diff_q, new_q;

	// 実際に回転を確認する関節番号
	int joint_no = -1;
	// 回転倍率
	float scale_ratio = 0.0f;

	// 各主要関節ごとにモーションワーピング後のキー時刻の姿勢を変形する
	for(int i=0; i<NUM_PRIMARY_JOINTS; i++)
	{
		// 関節番号のリセット
		joint_no = -1;

		// 実際に回転を確認する関節番号を取得
		// --- 足 (Leg) ---
		if (i == 6) // JOI_R_HIP (右股関節)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_HIP);
			scale_ratio = furi[0];
		}
		else if (i == 8) // JOI_R_KNEE (右膝)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_KNEE);
			scale_ratio = furi[0];
		}
		else if (i == 7) // JOI_L_HIP (左股関節)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_HIP);
			scale_ratio = furi[1];
		}
		else if (i == 9) // JOI_L_KNEE (左膝)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_KNEE);
			scale_ratio = furi[1];
		}

		// --- 腕 (Arm) ---
		else if (i == 0) // JOI_R_SHOULDER (右肩)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_SHOULDER);
			scale_ratio = furi[2];
		}
		else if (i == 2) // JOI_R_ELBOW (右肘)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_ELBOW);
			scale_ratio = furi[2];
		}
		else if (i == 1) // JOI_L_SHOULDER (左肩)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_SHOULDER);
			scale_ratio = furi[3];
		}
		else if (i == 3) // JOI_L_ELBOW (左肘)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_ELBOW);
			scale_ratio = furi[3];
		}

		else if (i == 12)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_BACK);
			// 回転倍率の設定
			scale_ratio = furi[5];
		}
		else if (i == 13)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_NECK);
			// 回転倍率の設定
			scale_ratio = furi[6];
		}

		// ルート関節以外の回転処理
		// 関節が見つからない場合(-1)はスキップ
		if (joint_no == -1)
			continue;

		org_q.set(param.org_pose.joint_rotations[joint_no]);
		before_q.set(before_posture.joint_rotations[joint_no]);

		// diff = org * before_inv
		// before_inv を計算 (共役クォータニオン)
		before_inv_q.x = -before_q.x;
		before_inv_q.y = -before_q.y;
		before_inv_q.z = -before_q.z;
		before_inv_q.w = before_q.w;
		// diff = org * before_inv
		diff_q.mul(org_q, before_inv_q);

		// scaled_diff = slerp(identity, diff, scale) = diff^scale
		// 内積(dot)を計算
		float dot_product = identity_q.x * diff_q.x + identity_q.y * diff_q.y + identity_q.z * diff_q.z + identity_q.w * diff_q.w;
		// 最短経路を保証
		if (dot_product < 0.0f)
		{
			diff_q.negate();
		}
		// 補間（SLERP）
		scaled_diff_q.interpolate(identity_q, diff_q, scale_ratio);

		// new = scaled_diff * org
		new_q.mul(scaled_diff_q, before_q);

		param.key_pose.joint_rotations[joint_no].set(new_q);
	}

	// ルート位置（移動量）のスケーリング処理
	// 足（右足:furi[0], 左足:furi[1]）のフリレベルの平均を移動倍率とする
	float move_scale = (furi[0] + furi[1]) / 2.0f;

	// 元の動作における移動ベクトル（速度）を計算
	// (現在のフレームの位置 - 1フレーム前の位置)
	Vector3f root_velocity;
	root_velocity = param.org_pose.root_pos - before_posture.root_pos;

	// 移動ベクトルに倍率を掛ける（足を1.5倍振るなら、移動も1.5倍にする）
	root_velocity = root_velocity * move_scale;

	// 1フレーム前の位置に、補正した移動ベクトルを足して新しい位置にする
	param.key_pose.root_pos = before_posture.root_pos + root_velocity;
	delete human_body;
}

//
//	回転行列からオイラー角への変換
//
void QuatToEulerYXZ(const Quat4f& q, double& y, double& x, double& z)
{
	// クォータニオンから回転行列へ変換
	Matrix3f mat;
	mat.set(q); // SimpleHuman.cpp 926行目付近の使用法に基づく

	// 回転行列からオイラー角(YXZ順)を抽出
	// R = Ry * Rx * Rz
	//     | CyCz+SySxSz   CzSxSy-CySz   CxSy |   | m00 m01 m02 |
	// R = | CxSz          CxCz          -Sx  | = | m10 m11 m12 |
	//     | CySxSz-CzSy   CyCzSx+SySz   CxCy |   | m20 m21 m22 |

	// m12 = -sin(x)
	x = asin(-mat.m12);

	// cos(x) が 0 でない場合 (ジンバルロックしていない場合)
	if (cos(x) != 0) {
		// m02 = cos(x)sin(y), m22 = cos(x)cos(y) -> tan(y) = m02/m22
		y = atan2(mat.m02, mat.m22);

		// m10 = cos(x)sin(z), m11 = cos(x)cos(z) -> tan(z) = m10/m11
		z = atan2(mat.m10, mat.m11);
	}
	else {
		// ジンバルロック時の処理 (x = +/- 90度)
		y = atan2(-mat.m20, mat.m00);
		z = 0;
	}

	// ラジアンから度数法へ変換
	x = x * 180.0 / M_PI;
	y = y * 180.0 / M_PI;
	z = z * 180.0 / M_PI;
}



//
//  動作変形（動作ワーピング）の適用後の動作を生成
//

Motion *  GenerateDeformedMotion( const MotionWarpingParam & deform, const Motion & motion, const vector<DistanceParam>& distance, float kire, float* furi)
{
	Motion *  deformed = NULL;

	// 動作変形前の動作を生成
	deformed = new Motion( motion );

	// 計算用変数
	TimeWarpingParam current_time_param;
	MotionWarpingParam current_motion_param = deform; // 初期値として引数を使用
	float current_before_time = 0.0f;
	Posture temp_posture(motion.body);

	// 各フレームの姿勢を変形
	for (int i = 0; i < motion.num_frames; i++)
	{
		float t = motion.interval * i;

		// パラメータを現在時刻 t に合わせて更新 (Animation関数内の処理を再現)
		InitTimeDeformationParameter(t, const_cast<vector<DistanceParam>&>(distance), current_time_param, const_cast<Motion&>(motion), kire);
		InitDeformationParameter(t, const_cast<vector<DistanceParam>&>(distance), current_motion_param, current_time_param, const_cast<Motion&>(motion), furi);

		// タイムワーピング適用後の姿勢を計算
		ApplyTimeWarping(t, current_time_param, motion, current_before_time, temp_posture);

		// モーションワーピングを適用し、結果を deformed->frames[i] に格納
		ApplyMotionDeformation(t, current_motion_param, const_cast<Motion&>(motion), temp_posture, current_time_param, deformed->frames[i]);
	}
	// 動作変形後の動作を返す
	return  deformed;
}



//
//  動作変形（動作ワーピング）の適用後の姿勢の計算
// （変形適用の重み 0.0～1.0 を返す）
//
float  ApplyMotionDeformation( float time, const MotionWarpingParam & deform, Motion& motion, Posture & input_pose, TimeWarpingParam time_param, Posture & output_pose )
{
	// タイムワーピング後の現在時刻と姿勢を計算
	//float warping_time = Warping(time, time_param);
	//motion.GetPosture(warping_time, input_pose);

	// タイムワーピング後の現在時刻
	float warping_time = 0.00;

	// タイムワーピング処理後の現在時刻の姿勢
	Posture warping_pose;

	// もし現在時刻にタイムワーピングを適用するなら
	if (time > time_param.warp_in_duration_time && time < time_param.warp_out_duration_time) {
		// タイムワーピング後の現在時刻を計算
		warping_time = Warping(time, time_param);
		// タイムワーピング後の現在時刻の姿勢を計算
		motion.GetPosture(warping_time, warping_pose);
	}
	else {
		warping_time = time;
		motion.GetPosture(time, warping_pose);
	}

	//printf("%f\n", warping_time);

	// 動作変形の範囲外であれば、入力姿勢を出力姿勢とする
	if ( ( warping_time < deform.key_time - deform.blend_in_duration ) || 
	     ( warping_time > deform.key_time + deform.blend_out_duration ) )
	{
		output_pose = input_pose;
		return  0.0f;
	}

	// 姿勢変形（動作ワーピング）の重みを計算
	float  ratio = 0.0f;
	if(warping_time <= deform.key_time)
		ratio = (warping_time - deform.blend_in_duration) / (deform.key_time - deform.blend_in_duration);
	if (warping_time > deform.key_time)
		ratio = 1.0f;


	// 姿勢変形（２つの姿勢の差分（dest - src）に重み ratio をかけたものを元の姿勢 org に加える ）
	PostureWarping( warping_pose, deform.org_pose, deform.key_pose, ratio, output_pose );

	return  ratio;
}


//
//  動作ワーピングの姿勢変形（２つの姿勢の差分（dest - src）に重み ratio をかけたものを元の姿勢 org に加える ）
//
void  PostureWarping( const Posture & org, const Posture & src, const Posture & dest, float ratio, Posture & p )
{
	// ３つの姿勢の骨格モデルが異なる場合は終了
	if ( ( org.body != src.body ) || ( src.body != dest.body ) || ( dest.body != p.body ) )
		return;

	// 骨格モデルを取得
	const Skeleton *  body = org.body;

	//計算用変数
	Quat4f q0, q1, q;
	Vector3f v;
	Matrix3f rot;

	// 各関節の回転を計算
	for ( int i = 0; i < body->num_joints; i++ )
	{
		rot.mulTransposeRight(dest.joint_rotations[i], src.joint_rotations[i]);
		rot.mul(rot, org.joint_rotations[i]);

		q0.set(org.joint_rotations[i]);
		q1.set(rot);

		//q0とq1の間を重みratioで補間してqに代入
		if (q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w < 0)
			q1.negate(q1);
		q.interpolate(q0, q1, ratio);
		p.joint_rotations[i].set(q);
	}

	// ルートの向きを計算
	rot.mulTransposeRight(dest.root_ori, src.root_ori);
	rot.mul(rot,org.root_ori);

	q0.set(org.root_ori);
	q1.set(rot);

	//q0とq1の間を重みratioで補間してqに代入
	if (q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w < 0)
		q1.negate(q1);
	q.interpolate(q0, q1, ratio);
	p.root_ori.set(q);

	// ルートの位置を計算
	v.sub(dest.root_pos, src.root_pos);
	v.scale(ratio);
	p.root_pos.add(v,org.root_pos);
}
