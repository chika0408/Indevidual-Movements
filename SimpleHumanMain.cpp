/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  メイン関数
**/


// GLUTフレームワーク＋アプリケーション基底クラスの定義を読み込み
#include "SimpleHumanGLUT.h"

// アプリケーションの定義を読み込み
#include "MotionPlaybackApp.h"
#include "KeyframeMotionPlaybackApp.h"
#include "ForwardKinematicsApp.h"
#include "PostureInterpolationApp.h"
#include "MotionInterpolationApp.h"
#include "MotionTransitionApp.h"
#include "MotionDeformationApp.h"
#include "MotionDeformationEditApp.h"
#include "InverseKinematicsCCDApp.h"

//#ifdef REPORT

// 応用アプリケーション（デモに含む）
#include "InverseKinematicsApp.h"
#include "InverseKinematicsJacobianApp.h"
#include "InverseKinematicsParticleApp.h"
#include "InverseKinematicsAnalyticalApp.h"
#include "InverseKinematicsStatisticalApp.h"
#include "MultipleMotionInterpolationApp.h"
#include "MotionAnalysisApp.h"
#include "StateMachineApp.h"
#include "MotionGraphApp.h"
#include "LocomotionPathApp.h"
#include "MotionSynthesisApp.h"
#include "CrowdApp.h"
#include "SkinApp.h"

// 応用アプリケーション（デモには含まない）
#include "CrowdSimulationLocomotionApp.h"
/*
#include "PoseAnalysisApp.h"
#include "MocapApp.h"
#include "MocapOptitrackApp.h"
#include "MocapNeuronApp.h"
#include "MocapNuiApp.h"
//#include "FightingGameApp.h"
//#include "ARMarkerApp.h"
//#include "MotionGraphApp.h"
//#include "DeepCrowdApp.h"
//#include "MotionSynthesisApp.h"
#include "DeepMotionApp.h"
#include "CrowdSimulationLocomotionApp.h"
//#include "MotionTransitionExApp.h"
#include "MotionDeformationExApp.h"
/**/

//#endif



//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// 全アプリケーションのリスト
	vector< class GLUTBaseApp * >    applications;

	// 優先アプリケーション（起動時に実行するアプリケーション）を登録
//	applications.push_back( new StateMachineApp() );
//	applications.push_back( new MotionDeformationExApp() );
//	applications.push_back( new MotionDeformationEditApp() );
//	applications.push_back( new MotionSynthesisApp() );
//	applications.push_back( new DeepMotionApp() );
//	applications.push_back( new CrowdSimulationLocomotionApp() );
	applications.push_back( new MotionTransitionApp() );
//	applications.push_back( new InverseKinematicsParticleApp() );
//	applications.push_back( new MotionGraphApp() );
//	applications.push_back( new InverseKinematicsStatisticalApp() );
//	applications.push_back( new MultipleMotionInterpolationApp() );
//	applications.push_back( new LocomotionPathApp() );
//	applications.push_back( new MotionSynthesisApp() );

	// 全アプリケーションを登録
	applications.push_back( new MotionPlaybackApp() );
	applications.push_back( new KeyframeMotionPlaybackApp() );
	applications.push_back( new ForwardKinematicsApp() );
	applications.push_back( new PostureInterpolationApp() );
	applications.push_back( new MotionInterpolationApp() );
	applications.push_back( new MotionDeformationEditApp() );
//	applications.push_back( new MotionTransitionApp() );
	applications.push_back( new InverseKinematicsCCDApp() );

//#ifdef REPORT
	// 応用アプリケーション（デモに含む）
	applications.push_back( new InverseKinematicsJacobianApp() );
	applications.push_back( new InverseKinematicsParticleApp() );
	applications.push_back( new InverseKinematicsAnalyticalApp() );
	applications.push_back( new InverseKinematicsStatisticalApp() );
//	applications.push_back( new InverseKinematicsApp() );
	applications.push_back( new MultipleMotionInterpolationApp() );
	applications.push_back( new MotionAnalysisApp() );
	applications.push_back( new LocomotionPathApp() );
	applications.push_back( new MotionSynthesisApp() );
	applications.push_back( new StateMachineApp() );
//	applications.push_back( new MotionGraphApp() );
	applications.push_back( new CrowdApp() );
	applications.push_back( new SkinApp() );

/*	// 応用アプリケーション（デモには含まない）
//	applications.push_back( new MotionTransitionExApp() );
//	applications.push_back( new MotionDeformationExApp() );
//	applications.push_back( new PoseAnalysisApp() );
	applications.push_back( new MocapApp() );
	applications.push_back( new MocapOptitrackApp() );
	applications.push_back( new MocapNeuronApp() );
	applications.push_back( new MocapNuiApp() );
//	applications.push_back( new MotionSynthesisApp() );
	applications.push_back( new DeepMotionApp() );
/*
//	applications.push_back( new FightingGameApp() );
//	applications.push_back( new ARMarkerApp() );
//	applications.push_back( new MotionGraphApp() );
//	applications.push_back( new DeepCrowdApp() );
//	applications.push_back( new CrowdSimulationLocomotionApp() );
/**/
//#endif
	
	// GLUTフレームワークのメイン関数を呼び出し（実行するアプリケーションのリストを指定）
	SimpleHumanGLUTMain( applications, argc, argv, NULL, 1280, 1024 );

	// GLUTフレームワークのメイン関数を呼び出し（単一のアプリケーションのみを実行する場合の例）
//	SimpleHumanGLUTMain( new MotionPlaybackApp(), argc, argv, "Motion Playback", 1280, 1024 );
// 
	// GLUTフレームワークのメイン関数を呼び出し（実行するアプリケーションを指定）
//	SimpleHumanGLUTMain( new InverseKinematicsApp(), argc, argv, "Inverse Kinematics" );
//	SimpleHumanGLUTMain( new MocapApp(), argc, argv, "Motion Capture" );
//	SimpleHumanGLUTMain( new MultipleMotionInterpolationApp(), argc, argv, "Multiple Motion Interpolation" );
//	SimpleHumanGLUTMain( new SkinApp(), argc, argv, "Skin Deformation" );
//	SimpleHumanGLUTMain( new PoseAnalysisApp(), argc, argv, "Pose Analysis" );
//	SimpleHumanGLUTMain( new CrowdApp(), argc, argv, "Crowd Simulation" );
}


