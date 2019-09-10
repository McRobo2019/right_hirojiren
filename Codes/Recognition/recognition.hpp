#ifndef RECOGNITION_H_
#define RECOGNITION_H_

#include "ev3api.h"
#include "parameter.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"
#include "util.hpp"

class Recognition {
public:
    explicit Recognition(const ev3api::ColorSensor& colorSensor,
		     ev3api::Motor& leftWheel,
		     ev3api::Motor& rightWheel,
		     ev3api::GyroSensor& gyro,
		     ev3api::SonarSensor& sonar);

  void  init(); //17.0.28 k-ota add
  void  run();
  void  det_line_rgb();

  void  wheel_odometry(float dT);
  void  average_dat(float dT); //20181008
  void  det_Movement();
  void  setSonarDistance(void);

  bool  stop_sys   = 0;
  int   linevalue  = 0;
  bool  green_flag = 0;

  float xvalue   = 0.0;//x座標推定値
  float yvalue   = 0.0;//y座標推定値

  float pre_50mm_x = 0.0;//50mm saki 20180512 kota
  float pre_50mm_y = 0.0;//50mm saki 20180512 kota

  float ave_x     = 0.0; //average_x 20181008
  float ave_y     = 0.0; //average_y 20181008

  float ave_vel_x = 0.0; //20181008
  float ave_vel_y = 0.0; //20181008


  float odo              = 0.0;//odometry
  float velocity         = 0.0;//Velocity
  float ave_velo         = 0.0;//Average_Velocity
  float ave_accel        = 0.0;//Average_Acceleration
  float pre_velo_0p5sec  = 0.0;//prediction vekicity 0.5 sec latar

  float left_wheel_velocity  = 0.0;
  float right_wheel_velocity = 0.0;
  float omega                = 0.0;


  
  int   encR      = 0;//右側タイヤ角度
  int   encL      = 0;
  int   pre_encR  = 0;//右側タイヤ角度
  int   pre_encL  = 0;

  float wheel_rotational_speed = 0;
  float ave_wheel_rot_speed    = 0;
  float abs_ave_wheel_speed    = 0;
  float wheel_load             = 0;
  float ave_wheel_load         = 0;

  float yawrate   = 0;
  float abs_angle = 0;
  float ave_angle = 0;
  float ave_angle_500 = 0;
  
  //signals for robo movement
  bool  robo_stop       = 0;
  bool  robo_forward    = 0;
  bool  robo_back       = 0;
  bool  robo_turn_left  = 0;
  bool  robo_turn_right = 0;

  int    sonarDistance = 0; // 距離 [cm]
  int    pre_sonar_dis = 0; // 距離 [cm]
  bool   sonar_stop  = false;

  int color_r, color_b, color_g;


private:
  const ev3api::ColorSensor& mColorSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::GyroSensor& mGyro;
  ev3api::SonarSensor& mSonar;

  Average_125_Data *gAve_angle_dat      = new Average_125_Data();
  Average_125_Data *gAve_x_dat          = new Average_125_Data();
  Average_125_Data *gAve_y_dat          = new Average_125_Data();
  Average_125_Data *gAve_vel_x_dat      = new Average_125_Data();
  Average_125_Data *gAve_vel_y_dat      = new Average_125_Data();
  Average_125_Data *gAve_velo_dat       = new Average_125_Data();
  Average_125_Data *gAve_accel_dat      = new Average_125_Data();
  Average_125_Data *gAve_wheel_rot_dat  = new Average_125_Data();
  Average_125_Data *gAve_wheel_load_dat = new Average_125_Data();

  Average_500_Data *gAve_angle_500_dat  = new Average_500_Data();
  
  int8_t dColor_val[5]; //170814 ota signals for filter of color sensor value.

  float real_wheel;
  float relative_angle;

  float dif_x_ave_dat = 0.0;
  float old_x_ave_dat = 0.0;

  float dif_y_ave_dat = 0.0;
  float old_y_ave_dat = 0.0;

  float dif_angle_ave_dat = 0.0;
  float old_angle_ave_dat = 0.0;

  float dif_velocity_ave_dat = 0.0;
  float old_velocity_ave_dat = 0.0;


  float WheelAngVLt = 0;
  float WheelAngVRt = 0;
  float RoboVt      = 0;
  float RoboAngVt   = 0;
  int   sonar_counter = 0;
};

#endif  // RECOGNITION_H_
