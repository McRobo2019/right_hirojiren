/******************************************************************************
 *  Author: Kaoru Ota
 *****************************************************************************/

#ifndef EV3_APP_ANAGOBRAIN_H_
#define EV3_APP_ANAGOBRAIN_H_
#include "parameter.h"
#include "util.hpp"
//#include "line_tracer.hpp"
#include "navi.hpp"
#include "map_trace.hpp"

using namespace std;

class Judgment {
public:
	explicit Judgment();//コンストラクタ
	void init();
	void set_drive_mode_LT();
	void set_drive_mode_TK();
	void set_drive_mode_DB();
	void run();
	void set_in_data(bool    green_flag,
			 float   velocity,
			 float   pre_velo_0p5sec,
			 float   yawrate,
			 float   ave_angle,
			 int     robo_tail_angle,
			 bool    robo_stop,
			 bool    robo_forward,
			 bool    robo_back,
			 bool    robo_turn_left,
			 bool    robo_turn_right,
			 int16_t sonar_dis);

  Average_500_Data *gAve_line_val      = new Average_500_Data();
  Average_500_Data *gAve_yaw_angle_500 = new Average_500_Data(); //20181108
  Average_500_Data *gAve_x_500 = new Average_500_Data();
  Average_500_Data *gAve_y_500 = new Average_500_Data();

  float ave_line_val;
  float ave_yaw_angle_500; //20181108

  bool  line_trace_mode;

  int   target_velocity;  //20190620 ota
  float target_omega;      //20190620 ota

  int   forward;           //前進目標値
  float target_yaw_rate;   //目標ヨーレート
  float ref_tail_angle;    //尻尾角度
  bool  tail_stand_mode;    //倒立走行フラグ

  bool  on_line;
  bool  left_line;
  bool  right_line;
  bool  lost_line;
  bool  line_to_map;  //181108

  int   det_navi_log;

  bool re_start; //20181112

private:
  //    StrategyDet *gStrategyDet = new StrategyDet();
  //  Motion_Ctl *gMotion_Ctl = new Motion_Ctl();
  Line_Trace *gLine_Trace = new Line_Trace();
  Navi       *gNavi       = new Navi();
  Map_Trace  *gMap_Trace  = new Map_Trace();

  int   Mmode;
  bool  mGreen_flag;  //ライン検出値
  float mVelocity;        //速度
  float mPre_velo_0p5sec; //prediction vekicity 0.5 sec latar
  float mYawrate;       //ヨーレート
  float mAve_yaw_angle;

  int   mTail_angle;
  //signals for robo movement
  bool  mRobo_stop       = false;
  bool  mRobo_forward    = false;
  bool  mRobo_back       = false;
  bool  mRobo_turn_left  = false;
  bool  mRobo_turn_right = false;

  int16_t mSonar_dis;
  
  int   mMax_Forward;

  float mRef_Yawrate;
  float mMax_Yawrate;
  float mMin_Yawrate;

  float mRef_Omega;
  float mMax_Omega;
  float mMin_Omega;


  enum Zone{
    START_ZONE,
    START_BACK,
    FIRST_STRAIGHT_ZONE,
    ENTER_1ST_CORNER_ZONE,
    FIRST_CORNER_ZONE,
    SECOND_STRAIGHT_ZONE,
    ENTER_2ND_CORNER_ZONE,
    SECOND_CORNER_ZONE,
    THIRD_STRAIGHT_ZONE,
    THIRD_CORNER_ZONE,
    S_CORNER_ZONE,
    FOURTH_STRAIGHT_ZONE,
    FOURTH_CORNER_ZONE,
    ENTER_5TH_CORNER_ZONE,
    FIFTH_CORNER_ZONE,
    FIRST_GRAY_ZONE,
    LUG_ZONE,
    BACK_LUG_ZONE,
    SECOND_GRAY_ZONE,
    SEESAW_ZONE,
    GARAGE_ZONE,
    LOST
  };

  enum Drive_Mode{
    LINE_TRACE,
    TRACK,
    DEBUG
  };

  enum On_Line_Mode{
    ON_THE_LEFT_EDGE,
    CENTER_LINE,
    LEFT_THE_LINE,
    RIGHT_THE_LINE,
    UNKOWN
  };

  enum Test_Mode{
    MODE_00,
    MODE_01,
    MODE_02,
    MODE_03,
    MODE_04,
    MODE_05,
    MODE_06,
    MODE_07,
    MODE_08,
    MODE_09
  };



  Zone         ZONE;
  Drive_Mode   DRIVE_MODE;
  On_Line_Mode ON_LINE_MODE;
  On_Line_Mode PRE_ON_LINE_MODE;
  Test_Mode    TEST_MODE;
};

#endif  // EV3_APP_ANAGOBRAIN_H_
