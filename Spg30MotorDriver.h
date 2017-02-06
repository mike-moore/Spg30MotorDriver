///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  a motor control driver for the SPG30E-XXXk DC motor
///  for the Arduino platform.
///
/// @author
///         $Author: Mike Moore $
///
/// Contact: michael.moore@nasa.gov
///
/// Created on: February 5 2017
///
///////////////////////////////////////////////////////////////
#ifndef SPG30MOTORDRIVER_H
#define SPG30MOTORDRIVER_H

#include <Arduino.h>

class Spg30MotorDriver { 
 public:
  /// @brief Construct the driver by providing all Arduino pin assignments
  Spg30MotorDriver(uint_least8_t loopRateMillis, uint_least8_t motorPinA1, 
  	               uint_least8_t motorPinB1, uint_least8_t pwmPin, uint_least8_t encoderPinA,
  	               uint_least8_t encoderPinB);

  /// @brief Defines this motor driver's modes of operation.
  enum ControlModes
  {
      POSITION = 0,
      VELOCITY = 1,
      IDLE     = 2
  };
  /// @brief User settable mode. Defaults to IDLE. The user
  ///        should set this before attempting a PositionCmd
  ///        or VelocityCmd.
  ControlModes ControlMode;
  /// @brief Driver's primary update routine. This runs the
  ///        driver's internal state machine.
  void run();
  /// @brief User should set one of these commands after specifying 
  ///        which control mode they want to use.
  void PositionCmd(float positionCmd);
  void VelocityCmd(int_least8_t velocityCmd);
  /// @brief Function ther user can call to see if their commanded
  ///        position has been reached.
  bool ReachedPosition();
  void _isr_EncoderA();
  void _isr_EncoderB();
 private:
  void _computeMotorSpeed();
  void _pidControl();
  void _updatePid(uint_least8_t command, uint_least8_t targetValue, uint_least8_t currentValue);
  uint_least8_t _loopRateMillis;
  uint_least8_t _motorPinA1;
  uint_least8_t _motorPinB1;
  uint_least8_t _pwmPin;
  uint_least8_t _encoderPinA;
  uint_least8_t _encoderPinB;
  uint_least8_t _encoderCountsPerRev;
  volatile long _encoderCount;
  int_least8_t _velocityCmd;
  float _positionCmd;
  uint_least8_t _pwmCmd;
  int _measuredSpeed;
  unsigned long _lastMillis;
  float _Kp;
  float _Kd;
  bool _A_set;
  bool _B_set;
  bool _positionReached; 
};

inline bool Spg30MotorDriver::ReachedPosition(){
	return _positionReached;
}

inline void Spg30MotorDriver::VelocityCmd(int_least8_t velocityCmd){
	// TODO constrain to +- 35 RPM
	_velocityCmd = velocityCmd;
}

inline void Spg30MotorDriver::PositionCmd(float positionCmd){
	// TODO bounds check.
	_positionCmd = positionCmd;
}

#endif