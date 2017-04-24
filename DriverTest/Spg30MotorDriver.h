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
  	               uint_least8_t motorPinB1, uint_least8_t pwmPin, volatile long int& encoderCount);

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
  /// @brief Defines the different speeds that can be used
  ///        for position control.
  enum MotorSpeeds
  {
      STOP         = 0,
      VEL_LOW      = 100,
      VEL_MEDIUM   = 200,
      VEL_HIGH     = 255
  };
  /// @brief The motor speed used for position control.
  /// @note  This is irrelevant for velocity control.
  MotorSpeeds MotorSpeed;
  /// @brief Driver's primary update routine. This runs the
  ///        driver's internal state machine.
  void run();
  /// @brief User should set one of these commands after specifying 
  ///        which control mode they want to use.
  void PositionCmd(int16_t positionCmd);
  void VelocityCmd(int velocityCmd);
  /// @brief Functions the user can call to see if their commanded
  ///        position and velocity have been reached.
  bool ReachedPosition();
  bool ReachedVelocity();
 private:
  void _computeMotorSpeed();
  void _pidControl();
  void _updatePid();
  void _positionControl();
  void _motorBackward();
  void _motorForward();
  void _motorBrake();
  void _printMotorInfo();
  uint_least8_t _loopRateMillis;
  uint_least8_t _motorPinA1;
  uint_least8_t _motorPinB1;
  uint_least8_t _pwmPin;
  uint_least8_t _encoderCountsPerRev;
  volatile long & _encoderCount;
  long _countInit;
  long _tickNumber;
  int _velocityCmd;
  int16_t _positionCmd;
  uint_least8_t _pwmCmd;
  int _measuredSpeed;
  unsigned long _lastMillis;
  unsigned long _lastMilliPrint;
  float _Kp;
  float _Kd;
  bool _motorIsRunning; 
  bool _positionReached; 
  bool _velocityReached; 
};

inline bool Spg30MotorDriver::ReachedPosition(){
	return _positionReached;
}

inline bool Spg30MotorDriver::ReachedVelocity(){
  return _velocityReached;
}

inline void Spg30MotorDriver::VelocityCmd(int velocityCmd){
	// TODO constrain to +- 35 RPM
  if(_velocityReached && velocityCmd != _velocityCmd){
      _velocityCmd = velocityCmd;
      _velocityReached = false;
  }
}

inline void Spg30MotorDriver::PositionCmd(int16_t positionCmd){
  // - Logic here checks that we have a non-zero position command,
  //   AND that we're not already trying to reach a position, AND
  //   that we haven't received the same command we're currently executing.
  if (positionCmd!=0 && _positionReached && positionCmd!=_positionCmd){
    _positionCmd = positionCmd;
    _tickNumber = positionCmd;
    _countInit = _encoderCount;
    _positionReached = false;
  }
}

#endif