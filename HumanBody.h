/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  人体モデルの骨格の追加情報
**/

#ifndef  _HUMAN_BODY_H_
#define  _HUMAN_BODY_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
//#include "MotionInfoEx.h"


// プロトタイプ宣言
struct  LimbParameter;


// 主要な体節を表す列挙型
enum  PrimarySegmentType
{
	SEG_R_FOOT,      // 右足
	SEG_L_FOOT,      // 左足
	SEG_R_HAND,      // 右手
	SEG_L_HAND,      // 左手
	SEG_PELVIS,      // 腰
	SEG_CHEST,       // 胸
	SEG_HEAD,        // 頭
	NUM_PRIMARY_SEGMENTS
};

// 主要な関節を表す列挙型
enum  PrimaryJointType
{
	JOI_R_SHOULDER,  // 右肩
	JOI_L_SHOULDER,  // 左肩
	JOI_R_ELBOW,     // 右ひじ
	JOI_L_ELBOW,     // 左ひじ
	JOI_R_WRIST,     // 右手首
	JOI_L_WRIST,     // 左手首
	JOI_R_HIP,       // 右尻
	JOI_L_HIP,       // 左尻
	JOI_R_KNEE,      // 右膝
	JOI_L_KNEE,      // 左膝
	JOI_R_ANKLE,     // 右足首
	JOI_L_ANKLE,     // 左足首
	JOI_BACK,        // 腰骨
	JOI_NECK,        // 首骨
	NUM_PRIMARY_JOINTS
};

// 主要な部位（体節＋関節）を表す列挙型
enum  PrimaryBodyPartType
{
	BODY_SEG_PELVIS,      // 腰
	BODY_SEG_R_FOOT,      // 右足
	BODY_SEG_L_FOOT,      // 左足
	BODY_SEG_R_HAND,      // 右手
	BODY_SEG_L_HAND,      // 左手
	BODY_SEG_HEAD,        // 頭
	BODY_JOI_R_SHOULDER,  // 右肩
	BODY_JOI_L_SHOULDER,  // 左肩
	BODY_JOI_R_ELBOW,     // 右ひじ
	BODY_JOI_L_ELBOW,     // 左ひじ
	BODY_JOI_R_WRIST,     // 右手首
	BODY_JOI_L_WRIST,     // 左手首
	BODY_JOI_R_HIP,       // 右尻
	BODY_JOI_L_HIP,       // 左尻
	BODY_JOI_R_KNEE,      // 右膝
	BODY_JOI_L_KNEE,      // 左膝
	BODY_JOI_R_ANKLE,     // 右足首
	BODY_JOI_L_ANKLE,     // 左足首
	BODY_JOI_BACK,        // 腰骨
	BODY_JOI_NECK,        // 首骨
	NUM_PRIMARY_BODY_PARTS
};

// 手足を表す列挙型
enum  LimbType
{
	LIMB_RIGHT_LEG, // 右足
	LIMB_LEFT_LEG,  // 左足
	LIMB_RIGHT_ARM, // 右手
	LIMB_LEFT_ARM,  // 左手
	NUM_LIMBS
};


//
//  人体モデルの骨格の追加情報を表すクラス
//
class  HumanBody
{
  protected:
	// 骨格の基本情報
	
	// 対象の骨格情報
	const Skeleton *  skeleton;

  protected:
	// 骨格の追加情報
	 
	// 主要体節の番号
	int  primary_segments[ NUM_PRIMARY_SEGMENTS ];

	// 主要関節の番号
	int  primary_joints[ NUM_PRIMARY_JOINTS ];

	// 逆運動学計算（解析的手法）のための手足の情報
	LimbParameter *  limbs[ NUM_LIMBS ];

	// 体節の質量
	float *  segment_weights;

	// 体節の慣性モーメント行列
	Matrix3f *  segment_inertia;


  public:
	// コンストラクタ
	HumanBody();
	HumanBody( const Skeleton * body );

	// デストラクタ
	~HumanBody();

  public:
	// 情報設定

	// 対象の骨格情報の設定
	void  SetSkeleton( const Skeleton * body );
	
	// 主要な部位の体節・関節番号の設定（番号で設定）
	void  SetPrimarySegment( PrimarySegmentType segment, int no );
	void  SetPrimaryJoint( PrimaryJointType joint, int no );

	// 主要な部位の体節・関節番号の設定（名前で設定）
	void  SetPrimarySegment( PrimarySegmentType segment, const char * name );
	void  SetPrimaryJoint( PrimaryJointType joint, const char * name );

	// 逆運動学計算（解析的手法）のための手足の情報の設定
	void  SetLimbParameter( LimbType limb, LimbParameter * info );

	// 体節の質量・慣性モーメント行列の設定
	void  SetPhysicalProperties( const float * segment_weights, const Matrix3f * segment_inertia );

  public:
	// 情報取得

	// 対象の骨格情報の取得
	const Skeleton *  GetSkeleton() const { return  skeleton; }

	// 主要な部位の体節・関節番号の取得
	int  GetPrimarySegment( PrimarySegmentType segment ) const { return  primary_segments[ segment ]; }
	int  GetPrimaryJoint( PrimaryJointType joint ) const { return  primary_joints[ joint ]; }
	bool  GetPrimaryBodyPart( PrimaryBodyPartType body_part, int & segment_no, int & joint_no ) const;

	// 逆運動学計算（解析的手法）のための手足の情報の取得
	LimbParameter *  GetLimbParameter( LimbType limb ) const { return  limbs[ limb ]; }

	// 体節の質量・慣性モーメント行列の取得
	float  GetSegmentWeight( int no ) const { return  segment_weights ? segment_weights[ no ] : 0.0f; }
	const Matrix3f *  GetSegmentInertia( int no ) const { return  segment_inertia ? &segment_inertia[ no ] : NULL; }
};



#endif // _HUMAN_BODY_H_
