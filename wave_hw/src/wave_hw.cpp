#include "wave_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Float64.h>
#include <math.h>

namespace wave {

WAVE_HW::WAVE_HW(std::string right_wheel_port, std::string left_wheel_port, double right_wheel_ikv, double left_wheel_ikv, double tacho_pulses_per_revolution, int motor_poles, disp_pos_mode rotor_position_source, ros::NodeHandle nh) :
  _right_wheel_driver(right_wheel_port, nh, "right_wheel", rotor_position_source),
  _left_wheel_driver(left_wheel_port, nh, "left_wheel", rotor_position_source)
 {
    _pos[0]=0;
    _pos[1]=0;
    _vel[0]=0;
    _vel[1]=0;
    _cmd[0]=0;
    _cmd[1]=0;
    _eff[0]=0;
    _eff[1]=0;
    _right_wheel_ikv = right_wheel_ikv;
    _left_wheel_ikv = left_wheel_ikv;
    _tacho_pulses_per_revolution = tacho_pulses_per_revolution;
    _motor_poles = motor_poles;
    // Convert rad/s to RPM and multiply by number of poles to get ERPM
    _rad_per_sec_to_erpm_conversion_factor = (60/(2*M_PI))*_motor_poles;
    _tacho_conversion_factor = (2*M_PI)/_tacho_pulses_per_revolution;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("left_wheel_joint", &_pos[0], &_vel[0], &_eff[0]);
    _jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("right_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&_jnt_state_interface);

    //right_speed_pub = nh.advertise<std_msgs::Float64>("ref/right_speed",10);
    //left_speed_pub = nh.advertise<std_msgs::Float64>("ref/left_speed",10);

    right_sub = nh.subscribe("right_wheel/control", 10, &WAVE_HW::setRightCallback, this);
    left_sub = nh.subscribe("left_wheel/control", 10, &WAVE_HW::setLeftCallback, this);

    // connect and register the joint velocity interfaces
    hardware_interface::JointHandle pos_handle_a(_jnt_state_interface.getHandle("left_wheel_joint"), &_cmd[0]);
    _jnt_vel_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(_jnt_state_interface.getHandle("right_wheel_joint"), &_cmd[1]);
    _jnt_vel_interface.registerHandle(pos_handle_b);

    registerInterface(&_jnt_vel_interface);
    }

  void WAVE_HW::setLeftCallback(std_msgs::Float64 msg){
	//ROS_INFO("left: I heard: %f" , msg.data);
        left_request_dutyCycle = 0.0;
        double requestedERPM = msg.data;
        double left_voltage_in = _left_wheel_driver.getVoltageIn();
        if(abs(requestedERPM) > 50){
        left_request_dutyCycle = requestedERPM / (left_voltage_in * _left_wheel_ikv * _motor_poles * 2);
         ROS_INFO("Left request dutycycle : %f, requested ERPM left: %f", left_request_dutyCycle, requestedERPM);
       _left_wheel_driver.setDutyCycle(left_request_dutyCycle);
         }
	else {
        _left_wheel_driver.releaseMotor();
	}
   }

  void WAVE_HW::setRightCallback(std_msgs::Float64 msg){

    double right_voltage_in = _right_wheel_driver.getVoltageIn();
    right_request_dutyCycle = 0.0;
    double requestedERPM2 = msg.data;
    if (abs(requestedERPM2) > 50)
    {

      right_request_dutyCycle = requestedERPM2 / (right_voltage_in * _right_wheel_ikv * _motor_poles * 2);
      ROS_INFO("Right request dutycycle : %f, requested ERPM right: %f", right_request_dutyCycle,  requestedERPM2);
    //  ROS_INFO("Output vars right_voltage_in : %f, _right_wheel_ikv : %f, _motor_poles : %d", right_voltage_in, _right_wheel_ikv,_motor_poles);
      _right_wheel_driver.setDutyCycle(right_request_dutyCycle);

   } else {
      _right_wheel_driver.releaseMotor();
   }

}



  void WAVE_HW::read(const ros::Time& time, const ros::Duration& period)
  {
    ROS_DEBUG("Reading from hardware...");

    _pos[0] = _left_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _pos[1] = -_right_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _vel[0] = _left_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;
    _vel[1] = -_right_wheel_driver.getSpeed()/_rad_per_sec_to_erpm_conversion_factor;

    double encoderDisplacementLeft = _left_wheel_driver.getEncoderDisplacement();
    double encoderDisplacementRight = _right_wheel_driver.getEncoderDisplacement();

    ROS_DEBUG("Position: %f : %f", _pos[0], _pos[1]);
    ROS_DEBUG("Encoder position: %f : %f", encoderDisplacementLeft, encoderDisplacementRight);
    ROS_DEBUG("Velocity: %f : %f", _vel[0], _vel[1]);
    ROS_DEBUG("Commands: %f : %f", _cmd[0], _cmd[1]);
    ROS_DEBUG("Effort: %f : %f", _eff[0], _eff[1]);
  }

  void WAVE_HW::write(const ros::Time& time, const ros::Duration& period)

  {
//    std_msgs::Float64 speed;
//    ROS_DEBUG("Writing to hardware...");
//    double requestedERPM = _rad_per_sec_to_erpm_conversion_factor * _cmd[0];
//    speed.data = requestedERPM;
//    left_speed_pub.publish(speed);
//    ROS_INFO("left %f",requestedERPM); 
//    double requestedERPM2 = -_rad_per_sec_to_erpm_conversion_factor * _cmd[1];
//    speed.data = requestedERPM2;
//    right_speed_pub.publish(speed);


//    ROS_DEBUG("Writing to hardware...");
//    double left_voltage_in = _left_wheel_driver.getVoltageIn();
//    double left_request_dutyCycle = 0.0;

//    if (_cmd[0] != 0.0)
//    {
//      double requestedERPM = _rad_per_sec_to_erpm_conversion_factor * _cmd[0];
//      ROS_DEBUG("Requested ERPM left: %f", requestedERPM);
//      left_request_dutyCycle = requestedERPM / (left_voltage_in * _left_wheel_ikv * _motor_poles * 2);
//      ROS_DEBUG("Left request dutycycle : %f", left_request_dutyCycle);
//       _left_wheel_driver.setDutyCycle(left_request_dutyCycle);
//    }
//    else
//    {
//      _left_wheel_driver.releaseMotor();
//    }

//    double right_voltage_in = _right_wheel_driver.getVoltageIn();
//    double right_request_dutyCycle = 0.0;
//    if (_cmd[1] != 0.0)
//    {
//      double requestedERPM = -_rad_per_sec_to_erpm_conversion_factor * _cmd[1];
//      ROS_INFO("Requested ERPM right: %f", requestedERPM);
//      right_request_dutyCycle = requestedERPM / (right_voltage_in * _right_wheel_ikv * _motor_poles * 2);
//      ROS_INFO("Right request dutycycle : %f", right_request_dutyCycle);
//      _right_wheel_driver.setDutyCycle(right_request_dutyCycle);
//    }
//    else
//    {
//      _right_wheel_driver.releaseMotor();
//    }


  }

}
