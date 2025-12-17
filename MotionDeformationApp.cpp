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

// csvファイル作成のため
#include <fstream>
#include <iostream>

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <numeric> // std::accumulate用
#include <cmath>   // std::pow用

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
	primary_segment_names[SEG_R_FOOT] = "RightFoot";
	primary_segment_names[SEG_L_FOOT] = "LeftFoot";
	primary_segment_names[SEG_R_HAND] = "RightHand";
	primary_segment_names[SEG_L_HAND] = "LeftHand";
	primary_segment_names[SEG_PELVIS] = "Hips";
	primary_segment_names[SEG_CHEST] = "Spine3"; //chest
	primary_segment_names[SEG_HEAD] = "Neck";

	// 末端部位の移動距離の合計をフレーム毎に配列として出力
	CheckDistance(*motion, distanceinfo, primary_segment_names);

	// 確認のためにdistanceinfoをcsvファイルに書き出す
	//std::ofstream outputfile("distanceinfo_output.csv");
	//for (auto&& b : distanceinfo) {
	//	outputfile << b.distanceadd;
	//	outputfile << ',';
	//	outputfile << b.move_amount;
	//	outputfile << '\n';
	//}
	//outputfile.close();

	// HumanBodyのセットアップ
	if (motion && motion->body) {
		// 既存のmotion->bodyからHumanBodyを作成
		my_human_body = new HumanBody(motion->body);

		// 部位名の登録（既存コードと同じ定義を使用）
		const char* primary_segment_names[NUM_PRIMARY_SEGMENTS];
		primary_segment_names[SEG_R_FOOT] = "RightFoot";
		primary_segment_names[SEG_L_FOOT] = "LeftFoot";
		primary_segment_names[SEG_R_HAND] = "RightHand";
		primary_segment_names[SEG_L_HAND] = "LeftHand";
		primary_segment_names[SEG_PELVIS] = "Hips";
		primary_segment_names[SEG_CHEST] = "Spine3";
		primary_segment_names[SEG_HEAD] = "Neck";

		for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) {
			my_human_body->SetPrimarySegment((PrimarySegmentType)i, primary_segment_names[i]);
		}
	}
	else {
		my_human_body = NULL;
	}

	// CSVファイルを開く（上書きモード）
	csv_file.open("deformed_motion_data.csv");
	// ヘッダー行を出力
	if (csv_file.is_open()) {
		csv_file << "Time,Frame,RightFoot_Dist,LeftFoot_Dist,RightHand_Dist,LeftHand_Dist,Pelvis_Dist,Chest_Dist,Head_Dist,Total_Dist" << std::endl;
	}

	// 前フレーム座標の初期化
	for (int i = 0; i < NUM_PRIMARY_SEGMENTS; i++) {
		prev_segment_positions[i] = Point3f(0, 0, 0);
	}

	// フリレベル・キレレベルの設定
	furi[0] = 1.0f;
	furi[1] = 1.0f;
	furi[2] = 1.0f;
	furi[3] = 1.0f;
	furi[4] = 1.0f;
	furi[5] = 20.0f;
	furi[6] = 20.0f;
	kire = 0.9f;

	// 編集中のレベルの設定
	selected_param = 0;

	// ねじれの計算
	double test1 = CalcChestVal(motion);

	// 動作変形情報の初期化
	//InitParameter();

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
	if (second_curr_posture)
	{
		glPushMatrix();

		//steplong_Char00の腰の位置が違ったので強引に修正
		glTranslatef( 0.0f, 0.0f, 0.3f ); 

		glEnable(GL_BLEND);
		glColor4f(1.0f, 0.0f, 1.0f, 0.5f);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
		DrawPosture(*second_curr_posture);
		DrawPostureShadow(*second_curr_posture, shadow_dir, shadow_color);
		glDisable(GL_BLEND);

		glPopMatrix();
	}

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
	if (key == '[' || key == ']')
	{
		float delta = (key == ']') ? 1.0f : -1.0f; // 変化量

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

	// r キーでリセット
	if ( key == 'r' )
		Start();
	
	// o キーで変形後の動作を保存
	if ( key == 'o' )
	{
		// 変形後の動作をBVH動作ファイルとして保存
		//SaveDeformedMotionAsBVH( "deformed_motion.bvh" );
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
		LoadBVH("radio_new_8_Char00.bvh");
		LoadSecondBVH("radio_long_8_Char00.bvh");

		//LoadSecondBVH("steplong_Char00.bvh");
		//LoadSecondBVH("radio_long_3_Char00.bvh");
		//LoadSecondBVH("fight_punch_key.bvh");
		//LoadSecondBVH("radio_long_2_Char00.bvh"); //4
		if ( !motion )
			return;
	}

	// 以下、他のテストケースを追加する
	else if ( no == 1 )
	{
	}
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
//  変形後の動作をBVH動作ファイルとして保存
//
/*
void  MotionDeformationApp::SaveDeformedMotionAsBVH( const char * file_name )
{
	// 動作変形適用後の動作を生成
	Motion *  deformed_motion = GenerateDeformedMotion( deformation, *motion );

	// 動作変形適用後の動作を保存
	// ※省略（各自作成）


	// 変形適用後の動作を削除
	delete  deformed_motion;
}
*/

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

double CalcChestVal(const Motion* motion)
{
	if (!motion) return 0.0;

	// 1. 関節の特定
	// HumanBodyクラスを使って関節IDを取得します
	HumanBody human_body(motion->body);

	// 一般的なBVHでの名前に対応（RightArmが右肩、LeftArmが左肩に対応することが多い）
	// お使いのボーン名に合わせて変更してください ("RightShoulder" など)
	human_body.SetPrimaryJoint(JOI_R_SHOULDER, "RightArm");
	human_body.SetPrimaryJoint(JOI_L_SHOULDER, "LeftArm");

	int rShldrIdx = human_body.GetPrimaryJoint(JOI_R_SHOULDER);
	int lShldrIdx = human_body.GetPrimaryJoint(JOI_L_SHOULDER);

	// 関節が見つからない場合のフォールバック（名前が違う場合など）
	if (rShldrIdx == -1 || lShldrIdx == -1) {
		// 別の名前候補でも試す例
		human_body.SetPrimaryJoint(JOI_R_SHOULDER, "RightShoulder");
		human_body.SetPrimaryJoint(JOI_L_SHOULDER, "LeftShoulder");
		rShldrIdx = human_body.GetPrimaryJoint(JOI_R_SHOULDER);
		lShldrIdx = human_body.GetPrimaryJoint(JOI_L_SHOULDER);

		if (rShldrIdx == -1 || lShldrIdx == -1) return 0.0;
	}

	std::vector<double> torsionValues;
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
		double zDiff = rPos.z - lPos.z;

		torsionValues.push_back(zDiff);
	}

	// 3. 分散を計算
	if (torsionValues.empty()) return 0.0;

	// 平均値
	double sum = std::accumulate(torsionValues.begin(), torsionValues.end(), 0.0);
	double mean = sum / torsionValues.size();

	// 分散
	double sqSum = 0.0;
	for (const double val : torsionValues) {
		sqSum += std::pow(val - mean, 2);
	}
	double variance = sqSum / torsionValues.size();

	return variance;
}


//
// 末端部位の移動距離を測定
//
void CheckDistance(const Motion& motion, vector<DistanceParam> & param, const char ** segment_names)
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

	// distanceaddのフレーム間の差分（＝微分）
	vector<float> distanceadd_diff;
	for(int i = 1; i < param.size(); i++)
		distanceadd_diff.push_back(param[i].distanceadd - param[i-1].distanceadd);

	// 極大値を取るときのフレーム番号
	vector<int> distance_ex_plus;
	// 極小値を取るときのフレーム番号
	vector<int> distance_ex_minus;

	for (int i = 5; i < distanceadd_diff.size()-5; i++)
	{
		// 極大値を探す
		if (distanceadd_diff[i] < 0 && distanceadd_diff[i - 1] > 0)
		{
			// 正負が一致しているかどうかを確認するフラグ(一致しなかったらtrue)
			bool distanceadd_diff_check = false;
			
			// 前後3フレームの微分の正負が一致しないと極値と認めない(4か3)
			for (int j = 1; j < 3; j++)
			{
				if(distanceadd_diff[i-j] < 0)
					distanceadd_diff_check = true;
				if(distanceadd_diff[i+j] > 0)
					distanceadd_diff_check = true;
			}

			// 10フレーム空いていないと極値と認めない
			for (int j = 0; j < distance_ex_plus.size(); j++)
			{
				if (i - 50 < distance_ex_plus[j])
					distanceadd_diff_check = true;
			}
			for (int j = 0; j < distance_ex_minus.size(); j++)
			{
				if (i - 50 < distance_ex_minus[j])
					distanceadd_diff_check = true;
			}
			
			if(distanceadd_diff_check == false)
				distance_ex_plus.push_back(i);
		}

		// 極小値を探す
		if (distanceadd_diff[i] > 0 && distanceadd_diff[i - 1] < 0)
		{
			// 正負が一致しているかどうかを確認するフラグ(一致しなかったらtrue)
			bool distanceadd_diff_check = false;
			
			// 前後3フレームの微分の正負が一致しないと極値と認めない(4か3)
			for (int j = 1; j < 3; j++)
			{
				if (distanceadd_diff[i - j] > 0)
					distanceadd_diff_check = true;
				if (distanceadd_diff[i + j] < 0)
					distanceadd_diff_check = true;
			}

			// 10フレーム空いていないと極値と認めない
			for (int j = 0; j < distance_ex_plus.size(); j++)
			{
				if (i - 50 < distance_ex_plus[j])
					distanceadd_diff_check = true;
			}
			for (int j = 0; j < distance_ex_minus.size(); j++)
			{
				if (i - 50 < distance_ex_minus[j])
					distanceadd_diff_check = true;
			}
			
			if (distanceadd_diff_check == false)
				distance_ex_minus.push_back(i);
		}
	}

		// 極小値・極大値の個数を少ない方に合わせる(オーバーフロー防止)
		int distance_ex_size;
		if(distance_ex_plus.size() >= distance_ex_minus.size())
			distance_ex_size = distance_ex_minus.size();
		else
			distance_ex_size = distance_ex_plus.size();

		// 閾値の設定に用いる極大値・極小値を定める
		// iは定めるフレーム
		for (int i = 0; i < distanceadd_diff.size(); i++)
		{
			int j=0, l=0;
			// jは極大値検索用
			if (distance_ex_plus.size() > 1)
			{
				for (j = 1; j < distance_ex_plus.size() - 1; j++)
				{
					if (i < distance_ex_plus[j])
					{
						if (fabs(i - distance_ex_plus[j]) > fabs(i - distance_ex_plus[j - 1]))
						{
							j = j - 1;
						}
						if (distance_ex_plus.size() < j)
							j = distance_ex_plus.size() - 1;
						break;
					}
				}
			}
			// lは極小値検索用
			if (distance_ex_minus.size() > 1)
			{
				for (l = 1; l < distance_ex_minus.size() - 1; l++)
				{
					if (i < distance_ex_minus[l])
					{
						if (fabs(i - distance_ex_minus[l]) > fabs(i - distance_ex_minus[l - 1]))
						{
							l = l - 1;
						}
						if (distance_ex_minus.size() < l)
							l = distance_ex_minus.size() - 1;
						break;
					}
				}
			}	
			param[i].move_amount =
				param[distance_ex_minus[l]].distanceadd + (param[distance_ex_plus[j]].distanceadd - param[distance_ex_minus[l]].distanceadd) / 5;
		}

		// 閾値を平滑化する
		for (int i = 5; i < param.size() - 5; i++)
		{
			for (int j = 1; j < 6; j++)
			{
				param[i].move_amount += param[i + j].move_amount;
				param[i].move_amount += param[i - j].move_amount;
			}
			param[i].move_amount = param[i].move_amount / 11;
		}

	for (int i = 0; i < param.size(); i++)
	{
		// movecheckの設定
		if(param[i].distanceadd < param[i].move_amount)
			param[i].movecheck = 0;
		else
			param[i].movecheck = 1;

		// movestartの設定
		if(param[i].movecheck == 1)
			param[i].move_start = true;
		else
		{
			param[i].move_start = false;
			if(i > 0){
				if(param[i-1].move_start)
					param[i].move_start = true;
			}
		}
	}
	delete human_body;
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
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.warp_in_duration_time = i * motion.interval;
					break;
				}
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
			for (int i = now_frame; i > 0; i--)
			{
				if (distance[i].movecheck == 1 && distance[i - 1].movecheck == 0)
				{
					param.blend_in_duration = i * motion.interval;
					break;
				}
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
			else if(distance[now_frame].movecheck == 1)
			{
				for (int i = now_frame; i < distance.size()-1; i++)
				{
					if (distance[i + 1].movecheck == 0)
					{
						param.key_time = i * motion.interval;
						break;
					}
				}
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
	primary_segment_names[SEG_R_FOOT] = "RightFoot";
	primary_segment_names[SEG_L_FOOT] = "LeftFoot";
	primary_segment_names[SEG_R_HAND] = "RightHand";
	primary_segment_names[SEG_L_HAND] = "LeftHand";
	primary_segment_names[SEG_PELVIS] = "Hips";
	primary_segment_names[SEG_CHEST] = "Spine3"; //chest
	primary_segment_names[SEG_HEAD] = "Neck";

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

		Point3f ee_pos;
		ee_pos = before_segment_positions[i];
		ee_pos += move_vec;

		ApplyInverseKinematicsCCD(param.key_pose, -1, seg_no, ee_pos);
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
	primary_joint_names[JOI_R_SHOULDER] = "RightArm";
	primary_joint_names[JOI_L_SHOULDER] = "LeftArm";
	primary_joint_names[JOI_R_ELBOW] = "RightForeArm";
	primary_joint_names[JOI_L_ELBOW] = "LeftForeArm";
	primary_joint_names[JOI_R_WRIST] = "RightHand";
	primary_joint_names[JOI_L_WRIST] = "LeftHand";
	primary_joint_names[JOI_R_HIP] = "RightUpLeg";
	primary_joint_names[JOI_L_HIP] = "LeftUpLeg";
	primary_joint_names[JOI_R_KNEE] = "RightLeg";
	primary_joint_names[JOI_L_KNEE] = "LeftLeg";
	primary_joint_names[JOI_R_ANKLE] = "RightFoot";
	primary_joint_names[JOI_L_ANKLE] = "LeftFoot";
	primary_joint_names[JOI_BACK] = "Spine";
	primary_joint_names[JOI_NECK] = "Neck";

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
		if (i == 6)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_HIP);
			// 回転倍率の設定
			scale_ratio = furi[0];
		}
		else if (i == 7)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_HIP);
			// 回転倍率の設定
			scale_ratio = furi[1];
		}
		else if (i == 0)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_R_SHOULDER);
			// 回転倍率の設定
			scale_ratio = furi[2];
		}
		else if (i == 1)
		{
			joint_no = human_body->GetPrimaryJoint(JOI_L_SHOULDER);
			// 回転倍率の設定
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
	delete human_body;
}


//
//  動作変形（動作ワーピング）の適用後の動作を生成
//
/*
Motion *  GenerateDeformedMotion( const MotionWarpingParam & deform, const Motion & motion )
{
	Motion *  deformed = NULL;

	// 動作変形前の動作を生成
	deformed = new Motion( motion );

	// 各フレームの姿勢を変形
	for ( int i = 0; i < motion.num_frames; i++ )
		ApplyMotionDeformation( motion.interval * i, deform, motion.frames[ i ], deformed->frames[ i ] );

	// 動作変形後の動作を返す
	return  deformed;
}
*/


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
	if(warping_time > deform.key_time)
		ratio = (deform.blend_out_duration - warping_time) / (deform.blend_out_duration - deform.key_time);


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
