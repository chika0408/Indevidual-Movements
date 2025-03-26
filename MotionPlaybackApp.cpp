/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作再生アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "MotionPlaybackApp.h"



//
//  コンストラクタ
//
MotionPlaybackApp::MotionPlaybackApp()
{
	app_name = "Motion Playback";

	motion = NULL;
	curr_posture = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	frame_no = 0;
}


//
//  デストラクタ
//
MotionPlaybackApp::~MotionPlaybackApp()
{
	// 骨格・動作・姿勢の削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;
}


//
//  初期化
//
void  MotionPlaybackApp::Initialize()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Initialize();

	// サンプルBVH動作データを読み込み
	//LoadBVH("fight_punch.bvh");
	LoadBVH( "radio_middle_1_Char00.bvh" );
}


//
//  開始・リセット
//
void  MotionPlaybackApp::Start()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Start();

	// アニメーション再生のための変数の初期化
	on_animation = true;
	animation_time = 0.0f;
	frame_no = 0;

	// アニメーション再生処理（姿勢の初期化）
	Animation( 0.0f );
}


//
//  画面描画
//
void  MotionPlaybackApp::Display()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Playback" );
	char  message[64];
	if ( motion )
		sprintf( message, "%.2f (%d)", animation_time, frame_no );
	else
		sprintf( message, "Press 'L' key to Load a BVH file" );
	DrawTextInformation( 1, message );
}


//
//  キーボードのキー押下
//
void  MotionPlaybackApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// s キーでアニメーションの停止・再開
	if ( key == 's' )
		on_animation = !on_animation;

	// w キーでアニメーションの再生速度を変更
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

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

	// l キーで再生動作の変更
	if ( key == 'l' )
	{
		// ファイルダイアログを表示してBVHファイルを選択・読み込み
		OpenNewBVH();
	}
}


//
//  アニメーション処理
//
void  MotionPlaybackApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
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
	motion->GetPosture( animation_time, *curr_posture );
}



//
//  補助処理
//


//
//  BVH動作ファイルの読み込み、動作・姿勢の初期化
//
void  MotionPlaybackApp::LoadBVH( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );
	
	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion )
		return;

	// 現在使用している骨格・動作・姿勢を削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// 動作再生に使用する動作・姿勢を初期化
	motion = new_motion;
	curr_posture = new Posture( motion->body );

	// 動作再生開始
	Start();
}


//
//  ファイルダイアログを表示してBVH動作ファイルを選択・読み込み
//
void  MotionPlaybackApp::OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// ファイルダイアログの設定
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof( OPENFILENAME ) );
	open_file.lStructSize = sizeof( OPENFILENAME );
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select a BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if ( ret )
	{
		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH( file_name );

		// 動作再生の開始
		Start();
	}
#endif // WIN32
}


