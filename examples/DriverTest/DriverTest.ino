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
    BWD_VEL_TEST = 3
};
TestCases TestNumber;

void setup() {                       
    Serial.begin(115600);
    // Very Important - attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(encoderPinA), ISR_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoderB, CHANGE);
    TestNumber = FWD_POS_TEST;
}

void loop() {
    switch(TestNumber)
    {
        case FWD_POS_TEST:
            // motorController.ControlMode = Spg30MotorDriver::POSITION; 
            // motorController.PositionCmd(720.0); 
            // motorController.run();
            // if (motorController.ReachedPosition){
                 TestNumber = BWD_POS_TEST;
            // }
        break;

        case BWD_POS_TEST:
            // motorController.ControlMode = Spg30MotorDriver::POSITION; 
            // motorController.PositionCmd(-720.0); 
            // motorController.run();
            // if (motorController.ReachedPosition){
                  TestNumber = FWD_VEL_TEST;
            // }
        break;

        case FWD_VEL_TEST:
            motorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            motorController.VelocityCmd(30.0); 
            motorController.run();
            delay(5000);
            TestNumber = BWD_VEL_TEST;
        break;

        case BWD_VEL_TEST:
            motorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            motorController.VelocityCmd(-30.0); 
            motorController.run();
            delay(5000);
            TestNumber = FWD_POS_TEST;
        break;

        default:
            /// - invalid state - go back to IDLE
            TestNumber = FWD_POS_TEST;
        break;
    }
}

void ISR_encoderA(){
    motorController._isr_EncoderA();
}

void ISR_encoderB(){
    motorController._isr_EncoderB();
}
