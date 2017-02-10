#include <Spg30MotorDriver.h>

// Encoder variables
enum PinAssignments {
    encoderPinA = 2,
    encoderPinB = 3,
    motorPwmPin = 9,
    motorPinA1  = 10,
    motorPinB1  = 11
};

// Control loop rate.
uint_least8_t loopTimeMillis = 100;

// Construct the motor driver for use.
Spg30MotorDriver motorController(loopTimeMillis, motorPinA1, motorPinB1, 
                                  motorPwmPin, encoderPinA, encoderPinB);

// Test cases ran in this driver test sketch.
enum TestCases
{
    FWD_POS_TEST = 0,
    BWD_POS_TEST = 1,
    FWD_VEL_TEST = 2,
    ZERO_VEL_TEST = 3,
    BWD_VEL_TEST = 4
};
TestCases TestNumber;

void setup() {                       
    Serial.begin(115600);
    digitalWrite(encoderPinA, HIGH);
    digitalWrite(encoderPinB, HIGH);
    // Very Important - attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoderB, CHANGE);
    TestNumber = FWD_POS_TEST;
}

void loop() {
    switch(TestNumber)
    {
        case FWD_POS_TEST:
            motorController.ControlMode = Spg30MotorDriver::POSITION; 
            motorController.PositionCmd(720); 
            motorController.run();
            if (motorController.ReachedPosition()){
                 Serial.println("FWD POSITION TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING BWD POSITION TEST");
                 TestNumber = BWD_POS_TEST;
            }
        break;

        case BWD_POS_TEST:
            motorController.ControlMode = Spg30MotorDriver::POSITION; 
            motorController.PositionCmd(-720); 
            motorController.run();
            if (motorController.ReachedPosition()){
                 Serial.println("BWD POSITION TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING FWD VELOCITY TEST");
                 TestNumber = FWD_VEL_TEST;
            }
        break;

        case FWD_VEL_TEST:
            motorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            motorController.VelocityCmd(25); 
            motorController.run();
            if (motorController.ReachedVelocity()){
                 Serial.println("FWD VELOCITY TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING ZERO VELOCITY TEST");
                 TestNumber = ZERO_VEL_TEST;
            }
        break;

        case ZERO_VEL_TEST:
            motorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            motorController.VelocityCmd(0); 
            motorController.run();
            if (motorController.ReachedVelocity()){
                 Serial.println("ZERO VELOCITY TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING BWD VELOCITY TEST");
                 TestNumber = BWD_VEL_TEST;
            }
        break;

        case BWD_VEL_TEST:
            motorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            motorController.VelocityCmd(-25); 
            motorController.run();
            if (motorController.ReachedVelocity()){
                 Serial.println("BWD VELOCITY TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING FWD POSITION TEST");
                 TestNumber = FWD_POS_TEST;
            }
        break;

        default:
            /// - invalid state - go back to IDLE
            TestNumber = FWD_POS_TEST;
        break;
    }
    delay(loopTimeMillis);
}

void ISR_encoderA(){
    motorController._isr_EncoderA();
}

void ISR_encoderB(){
    motorController._isr_EncoderB();
}
