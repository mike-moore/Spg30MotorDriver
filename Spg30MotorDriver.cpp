#include "Spg30MotorDriver.h"

Spg30MotorDriver::Spg30MotorDriver(uint_least8_t loopRateMillis, uint_least8_t motorPinA1,
                   uint_least8_t motorPinB1, uint_least8_t pwmPin, uint_least8_t encoderPinA,
  	               uint_least8_t encoderPinB) :
    ControlMode(IDLE),
    _loopRateMillis(loopRateMillis),
    _motorPinA1(motorPinA1),
    _motorPinB1(motorPinB1),
    _pwmPin(pwmPin),
    _encoderPinA(encoderPinA),
    _encoderPinB(encoderPinB),
    _encoderCountsPerRev(360),
    _encoderCount(0),
    _velocityCmd(0),
    _positionCmd(0.0),
    _pwmCmd(0),
    _measuredSpeed(0),
    _lastMillis(0),
    _Kp(0.4),
    _Kd(1.0),
    _A_set(false),
    _B_set(false),
    _positionReached(false)
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

void Spg30MotorDriver::run() {
    switch(ControlMode)
    {
        case POSITION:

        break;

        case VELOCITY:
            _pidControl();
        break;

        case IDLE:

        break;

        default:
            /// - invalid state - go back to IDLE
            ControlMode = IDLE;
        break;
    }
}

void Spg30MotorDriver::_computeMotorSpeed() {
	static long lastCount = 0;
	_measuredSpeed = ((_encoderCount - lastCount)*(60*(1000/_loopRateMillis)))/(_encoderCountsPerRev);
	lastCount = _encoderCount;
}

void Spg30MotorDriver::_pidControl() {
	if((millis() - _lastMillis) >= _loopRateMillis){
		_lastMillis = millis();
		_computeMotorSpeed();
		_updatePid(_pwmCmd, _velocityCmd, _measuredSpeed);
		analogWrite(_pwmPin, _pwmCmd);
	}
}

void Spg30MotorDriver::_updatePid(uint_least8_t command, uint_least8_t targetValue, uint_least8_t currentValue) {
	float pidTerm = 0;
	uint_least8_t error=0;                                  
	static uint_least8_t last_error=0;          
	if (_velocityCmd < 0.0){
		digitalWrite(_motorPinA1, HIGH);
		digitalWrite(_motorPinB1, LOW);                  
	}else {
		digitalWrite(_motorPinA1, LOW);
		digitalWrite(_motorPinB1, HIGH);    
	}
	error = abs(targetValue) - abs(currentValue); 
	pidTerm = (_Kp * error) + (_Kd * (error - last_error));                            
	last_error = error;
	_pwmCmd = constrain(command + uint_least8_t(pidTerm), 0, 255);
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

