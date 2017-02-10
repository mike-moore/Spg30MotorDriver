#include "Spg30MotorDriver.h"

Spg30MotorDriver::Spg30MotorDriver(uint_least8_t loopRateMillis, uint_least8_t motorPinA1,
                   uint_least8_t motorPinB1, uint_least8_t pwmPin, uint_least8_t encoderPinA,
  	               uint_least8_t encoderPinB) :
    ControlMode(IDLE),
    MotorSpeed(VEL_MEDIUM),
    _loopRateMillis(loopRateMillis),
    _motorPinA1(motorPinA1),
    _motorPinB1(motorPinB1),
    _pwmPin(pwmPin),
    _encoderPinA(encoderPinA),
    _encoderPinB(encoderPinB),
    _encoderCountsPerRev(360),
    _encoderCount(0),
    _countInit(0),
    _tickNumber(0),
    _velocityCmd(0),
    _positionCmd(0),
    _pwmCmd(0),
    _measuredSpeed(0),
    _lastMillis(0),
    _lastMilliPrint(0),
    _Kp(0.4),
    _Kd(1.0),
    _A_set(false),
    _B_set(false),
    _motorIsRunning(false),
    _positionReached(true),
    _velocityReached(true)
{
    /// - Initialize the Arduino pins for motor control and encoder readings.
    pinMode(_motorPinA1, OUTPUT);
    pinMode(_motorPinB1, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    digitalWrite(_encoderPinA, HIGH);
    digitalWrite(_encoderPinB, HIGH);
    analogWrite(_pwmPin, 0);
    digitalWrite(_motorPinA1, LOW);
    digitalWrite(_motorPinB1, HIGH);
}

void Spg30MotorDriver::run(){
    switch(ControlMode)
    {
        case POSITION:
            if(!_positionReached){
                _positionControl();
            }
            if(_motorIsRunning){
                if((abs(abs(_encoderCount)-abs(_countInit))) >= abs(_tickNumber)){
                    _positionReached = true;
                    _positionCmd = 0;
                    _encoderCount = 0;
                    _motorBrake();
                }
            }
        break;

        case VELOCITY:
            _pidControl();
        break;

        case IDLE:
            _motorBrake();
        break;

        default:
            /// - invalid state - go back to IDLE
            ControlMode = IDLE;
        break;
    }
}

void Spg30MotorDriver::_computeMotorSpeed(){
  	static long lastCount = 0;
  	_measuredSpeed = ((_encoderCount - lastCount)*(60*(1000/_loopRateMillis)))/_encoderCountsPerRev;
  	lastCount = _encoderCount;
}

void Spg30MotorDriver::_pidControl(){
  	if((millis() - _lastMillis) >= _loopRateMillis){
	  	_lastMillis = millis();
	  	_computeMotorSpeed();
	  	_updatePid();
	  	analogWrite(_pwmPin, _pwmCmd);
	  }
    _printMotorInfo();
}

void Spg30MotorDriver::_updatePid(){
    float pidTerm = 0;
    int error=0;                                  
	  static int last_error=0;          
	  if (_velocityCmd < 0.0){
	     	digitalWrite(_motorPinA1, HIGH);
	     	digitalWrite(_motorPinB1, LOW);                  
	  }else {
	      digitalWrite(_motorPinA1, LOW);
	     	digitalWrite(_motorPinB1, HIGH);    
	  }
	  error = abs(_velocityCmd) - abs(_measuredSpeed); 
	  pidTerm = (_Kp * error) + (_Kd * (error - last_error));                            
    last_error = error;
	  _pwmCmd = constrain(_pwmCmd + int(pidTerm), 0, 255);
}

void Spg30MotorDriver::_positionControl(){
    if(_positionCmd > 0){
        _motorForward();
    }else if(_positionCmd < 0){
        _motorBackward();
    }
}

void Spg30MotorDriver::_motorForward(){
   analogWrite(_pwmPin, MotorSpeed);
   digitalWrite(_motorPinA1, LOW);
   digitalWrite(_motorPinB1, HIGH);
   _motorIsRunning = true;
}

void Spg30MotorDriver::_motorBackward(){
   analogWrite(_pwmPin, MotorSpeed);
   digitalWrite(_motorPinA1, HIGH);
   digitalWrite(_motorPinB1, LOW);
   _motorIsRunning = true;
}

void Spg30MotorDriver::_motorBrake(){
   analogWrite(_pwmPin, STOP);
   digitalWrite(_motorPinA1, HIGH);
   digitalWrite(_motorPinB1, HIGH);
   _motorIsRunning = false;
}

void Spg30MotorDriver::_printMotorInfo(){  
  if((millis()-_lastMilliPrint) >= 500){                     
        _lastMilliPrint = millis();
        Serial.print("SP:");             Serial.println(_velocityCmd);  
        Serial.print("  RPM:");          Serial.println(_measuredSpeed);
        Serial.print("  PWM:");          Serial.println(_pwmCmd);
        Serial.println("");              
    }
}

// Interrupt on A changing state
void Spg30MotorDriver::_isr_EncoderA(){
    // Test transition
    _A_set = digitalRead(_encoderPinA) == HIGH;
    // and adjust counter + if A leads B
    _encoderCount += (_A_set != _B_set) ? +1 : -1;
}

// Interrupt on B changing state
void Spg30MotorDriver::_isr_EncoderB(){
    // Test transition
    _B_set = digitalRead(_encoderPinB) == HIGH;
    // and adjust counter + if B follows A
    _encoderCount += (_A_set == _B_set) ? +1 : -1;
}

