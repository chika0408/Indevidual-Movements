/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作再生アプリケーション
**/

#ifndef  _MOTION_PLAYBACK_APP_H_
#define  _MOTION_PLAYBACK_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"


//
//  動作再生アプリケーションクラス
//
class  MotionPlaybackApp : public GLUTBaseApp
{
  protected:
	// 動作再生の入力情報

	// 動作データ
	Motion *  motion;

  protected:
	// 動作再生のための変数

	// 現在の姿勢
	Posture *  curr_posture;

	// アニメーション再生中かどうかを表すフラグ
	bool  on_animation;

	// アニメーションの再生時間
	float  animation_time;

	// アニメーションの再生速度
	float  animation_speed;

	// 現在の表示フレーム番号
	int  frame_no;

  public:
	// コンストラクタ
	MotionPlaybackApp();

	// デストラクタ
	virtual ~MotionPlaybackApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );

  public:
	// 補助処理

	// BVH動作ファイルの読み込み、動作・姿勢の初期化
	void  LoadBVH( const char * file_name );

	// ファイルダイアログを表示してBVH動作ファイルを選択・読み込み
	void  OpenNewBVH();
};


#endif // _MOTION_PLAYBACK_APP_H_
