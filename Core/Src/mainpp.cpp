/*
 * mainpp.cpp
 *
 *  Created on: Apr 2, 2022
 *      Author: PC
 */

#include<mainpp.h>
#include<ros.h>
#include<std_msgs/Int32.h>
#include "turtlebot3_core_config.h"

//아두이노  millis 함수 대체 변수
extern int millis;


uint32_t nowTick = 0;
uint32_t pastTick = 0;

int num_test = 0;

/******************************************************************************
***************** void setup***************************************************
***************** void setup***************************************************
***************** void setup***************************************************
*******************************************************************************/

void setup()
{
	//노드 핸들러 제작
	nh.initNode();

	//보드레이트는 main.c에서 조절


    //subscriber 선언
    nh.subscribe(cmd_vel_sub);          //커맨드 속도 받기
    nh.subscribe(sound_sub);            //소리
    nh.subscribe(motor_power_sub);      //모터 파워 신호 받기?
    nh.subscribe(reset_sub);            //리셋 신호 받기

    //publisher 함수

    nh.advertise(imu_pub);              //////////////////////imu 센서값 주기, 수정해야할 것///
    nh.advertise(cmd_vel_rc100_pub);    //
    nh.advertise(odom_pub);             //오도메트리 정보
    nh.advertise(joint_states_pub);     //조인트? 상태 정보
    nh.advertise(battery_state_pub);    //배터리 상태 정보
    nh.advertise(mag_pub);              //마그네틱 정보
    nh.advertise(sensor_state_pub);     //센서 상태 주기
    nh.advertise(version_info_pub);     //버전 정보 주기


    //milis 구현하기: ms를 기록한다. 0.001초 체크하는 타이머 만들기

    //시작하는 시간 지정
    prev_update_time = millis;

    tf_broadcaster.init(nh);

     // Setting for Dynamixel motors
     //motor_driver.init(NAME);

     // Setting for IMU
     sensors.init();

     // Init diagnosis
    // diagnosis.init();

     // Setting for ROBOTIS RC100 remote controller and cmd_vel
     //controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

     // Setting for SLAM and navigation (odometry, joint states, TF)
     initOdom();

     initJointStates();

     prev_update_time = millis();
    //셋업 끝남을 알려주는 flag
    setup_end = true;
}

/*******************************************************************************
***************** void loop***************************************************
***************** void loop***************************************************
***************** void loop***************************************************
*******************************************************************************/
void loop()
{
	//t는 아마 현재시간?
	uint32_t t = millis;

	//curent_offset은 현재시간, current_time은 nh.now()(이건 알아봐야한다.)
	updateTime();

	//nh에 연결되면 nh.connected()가 1이 되는가보다.
	updateVariable(nh.connected());
	updateTFPrefix(nh.connected());

//////////////1000/30ms 마다 목표 속도 업데이트/////////
//////////////1000/30ms 마다 목표 속도 업데이트/////////
//////////////1000/30ms 마다 목표 속도 업데이트/////////
	 /* if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))   //CONTROL_MOTOR_SPEED_FREQUENCY=30
	  {
		//속도 제어신호 업데이트
		updateGoalVelocity();

		/////////////////500ms(모터 마다
		//tTime[6] clicked_state가 1일때 목표 속도가 모터에 적용된다.
		if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT)   {
			//////////////////////////////////////////////////////////////////휠 반지름, 휠 사이 간격, 모터 속도로 조절
			//motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity); => mecanumwheelControl_hj::movestop();
			//
		}
		else {
			//////////////////////////////////////////////////////////////////////////////휠 반지름, 휠 사이 간격, 모터 속도로 조절
			//motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity); => 모터 속도 고정이므로 생략가능?
			//
		}
		tTime[0] = t;
	  }*/

//////////////////////////IMU 퍼블리셔///////////
	//stm으로 imu 센서 값 받아와서 보내기
	//////////////////////////////////////////////터틀봇에서 ros로 보내는 데이터 형태 확인하기
	  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
	  {
	    publishImuMsg();
	    publishMagMsg();
	    tTime[3] = t;
	  }

	//******************************************그대로 쓸거***********************************************************
	//******************************************그대로 쓸거*********************************************************
	//******************************************그대로 쓸거********************************************************
	//1. 속도 퍼블리셔
	//2. 센서, 배터리, 주행 정보 퍼블리셔
	//3. 퍼블리셔 버전 정보?

	/////////////현재 속도 ros로 보내주는 퍼블리셔
/*	  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY)) //CMD_VEL_PUBLISH_FREQUENCY =30
	  {
	    publishCmdVelFromRC100Msg();
	    void publishCmdVelFromRC100Msg(void){
	      cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
	      cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

	      cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg); --> &cmd_vel_rc100_msg 정의에서 고정 속도 대입
	    }
	    tTime[1] = t;
	  }*/

	/////////////////////////////////////주행상태 퍼블리셔
	  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
	  {
	    publishSensorStateMsg();
	    publishBatteryStateMsg();
	    publishDriveInformation();
	    tTime[2] = t;
	  }

	/////////////////////////////////퍼블리시 버전 정보??
	  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
	  {
	    publishVersionInfoMsg();
	    tTime[4] = t;
	  }

	////////////////////////////////////////////////////////////여기서부터 모르겠음
	////////////////////////////////////////////////////////////여기서부터 모르겠음
	////////////////////////////////////////////////////////////여기서부터 모르겠음


	  // Send log message after ROS connection
	  //ROS 연결 후 로그 메세지를 보낸다.

	  // Receive data from RC100
	  //bool clicked_state = controllers.getRCdata(goal_velocity_from_rc100);

	 /* if (clicked_state == true)
	    tTime[6] = millis;*/

	  // Check push button pressed for simple test drive
	  //driveTest(diagnosis.getButtonPress(3000));

	  // Update the IMU unit
	  //IMU 센서 데이터 업데이트??
	  //sensors.updateIMU();

	  // TODO
	  // Update sonar d`ata 소나 데이터는 뭐지?
	  // sensors.updateSonar(t);

	  //ROS 연결 이후 자이로 센서 캘리브레이션
	  updateGyroCali(nh.connected());

	  // LED 상태 보이기
	  //diagnosis.showLedStatus(nh.connected()); //LED 필요 x

	  // 전압 데이터 업데이트
	  battery_state = 2;
/*diagnosis.updateVoltageCheck(setup_end) 자리에 밑에 값 입력 --> 정상이라고 가정하면 2
	  #define BATTERY_POWER_OFF                0
#define BATTERY_POWER_STARTUP            1
#define BATTERY_POWER_NORMAL             2
#define BATTERY_POWER_CHECK              3
#define BATTERY_POWER_WARNNING           4
*/

	  // Call all the callbacks waiting to be called at that point in time
	  //
	  nh.spinOnce();

	  // Wait the serial link time to process
	  waitForSerialLink(nh.connected());

}
/*******************************************************************************
***************** void loop end***************************************************
***************** void loop end***************************************************
***************** void loop end***************************************************
*******************************************************************************/



////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////섭스크라이브 콜백함수///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//cmd_vel_msg
//motor_power_msg
//sound_msg
//reset_msg
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*void updateGoalVelocity(void) -> 속도 고정
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];

  //sensors.setLedPattern(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
}*/

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
  //goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  if( goal_velocity_from_cmd[LINEAR]<MIN_LINEAR_VELOCITY) goal_velocity_from_cmd[LINEAR] =MIN_LINEAR_VELOCITY;
  if(goal_velocity_from_cmd[LINEAR]>MAX_LINEAR_VELOCITY) goal_velocity_from_cmd[LINEAR] =MAX_LINEAR_VELOCITY;

  //goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  if( goal_velocity_from_cmd[ANGULAR]<MIN_LINEAR_VELOCITY) goal_velocity_from_cmd[ANGULAR] =MIN_LINEAR_VELOCITY;
  if(goal_velocity_from_cmd[ANGULAR]>MAX_LINEAR_VELOCITY) goal_velocity_from_cmd[ANGULAR] =MAX_LINEAR_VELOCITY;


  tTime[6] = millis;
}

void soundCallback(const turtlebot3_msgs::Sound& sound_msg){
	//소리 안내도 되니까 주석
  //sensors.makeSound(sound_msg.value);
}

void motorPowerCallback(const std_msgs::Bool& power_msg){
  bool dxl_power = power_msg.data;
  /////////////////////////////////////////////////////////////////////모터 파워 데이터에 따른 동작 구현하기
  //motor_driver.setTorque(dxl_power); -> 토크설정 필요x ->true 설정
}

void resetCallback(const std_msgs::Empty& reset_msg){
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  ///////////////////////////////////////////////////////////////////////////////////////자이로센서 캘리브레이션
  //sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////publisher 함수///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// cmd_vel_rc100_msg (CMD Velocity data from RC100 : angular velocity, linear velocity)
// imu_msg (IMU data: angular velocity, linear acceleration, orientation)
// mag_msg (Magnetic data)
// sensor_state_msg (sensor_state: bumpers, cliffs, buttons, encoders, battery)
// version_info_msg (version info)
// battery_state_msg (battery_state)
// odom_pub, joint_states_pub (odometry, joint states, tf)
//

void publishCmdVelFromRC100Msg(void){ // => 속도 고정이므로 속도 지정
  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
}

void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  //모터드라이브 왼쪽 오른쪽 싱크가 맞으면 엔코더값을 각각 입력

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder); //엔코더값 평균낼지 우짤지 고민
  else
    return;

  sensor_state_msg.bumper = sensors.checkPushBumper();

  sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  sensor_state_msg.illumination = sensors.getIlluminationData();

  sensor_state_msg.button = sensors.checkPushButton();

  sensor_state_msg.torque = motor_driver.getTorque(); // 토크값 설정 필요없지만 퍼블리셔에 필요하므로 --> 근데 bool형이어서 전원켜지면 true로?

  sensor_state_pub.publish(&sensor_state_msg);
}

void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;

  battery_state_pub.publish(&battery_state_msg);
}

void publishDriveInformation(void)
{
  unsigned long time_now = millis;
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

//////////////////////////////////////////루프문 시작할 때마다 실행되는 코드////////////////////
//////////////////////////////////////////루프문 시작할 때마다 실행되는 코드////////////////////
//////////////////////////////////////////루프문 시작할 때마다 실행되는 코드////////////////////

//시간 갱신 함수
void updateTime()
{
  current_offset = millis;
  current_time = nh.now();
}

//Update variable (initialization)
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;

  if (isConnected)
  {
    if (variable_flag == false)
    {
    	//IMU센서를 시작하는 함수인것같음--> 없어도 됨
      //sensors.initIMU();

      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

//Update TF Prefix
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[60];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

//////////////////////////////// publishSensorStateMsg 할 때 엔코더값 업데이트////////////////////////////////
//////////////////////////////// publishSensorStateMsg 할 때 엔코더값 업데이트////////////////////////////////
//////////////////////////////// publishSensorStateMsg 할 때 엔코더값 업데이트////////////////////////////////

void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  //init_encoder는 initOdom에서 트루가 된다.
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/////////////////////////////publishDriveInformation할 때 오도메트리 캘리브레이션//////////////////////////
/////////////////////////////publishDriveInformation할 때 오도메트리 캘리브레이션//////////////////////////
/////////////////////////////publishDriveInformation할 때 오도메트리 캘리브레이션//////////////////////////
//---> 연구할만 하겠다.

bool calcOdometry(double diff_time){
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3],
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

////////////////////////////////////////opencr에 있는 버튼 눌렀을 때 회전하거나 직진한다.///////////////////////////////
////////////////////////////////////////opencr에 있는 버튼 눌렀을 때 회전하거나 직진한다.///////////////////////////////
////////////////////////////////////////opencr에 있는 버튼 눌렀을 때 회전하거나 직진한다.///////////////////////////////

/*
void driveTest(uint8_t buttons){
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  motor_driver.readEncoder(current_tick[LEFT], current_tick[RIGHT]);

  if (buttons & (1<<0))
  {
    move[LINEAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
    tTime[6] = millis;
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
    tTime[6] = millis;
  }

  if (move[LINEAR])
  {
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
      tTime[6] = millis;

    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= -0.7;
      tTime[6] = millis;
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}
*/

////////////////////////////////////루프문 마지막마다 실행하는 코드///////////////////////////////////
////////////////////////////////////루프문 마지막마다 실행하는 코드///////////////////////////////////
////////////////////////////////////루프문 마지막마다 실행하는 코드///////////////////////////////////


void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      HAL_Delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

//오도메트리 세팅하기
//리셋 콜백할때, 오도메트리 리셋
//변수 업데이트할때,
//

void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/


/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/

ros::Time rosNow()
{
  return nh.now();
}

