#include "Spg30MotorDriver.h"

// Encoder variables
enum RightMotorPinAssignments {
    mtrR_encoderPinA = 7,
    mtrR_encoderPinB = 6,
    mtrR_motorPwmPin = 9,
    mtrR_motorPinA1  = 11,
    mtrR_motorPinB1  = 10
};

enum LeftMotorPinAssignments {
    mtrL_encoderPinA = 4,
    mtrL_encoderPinB = 5,
    mtrL_motorPwmPin = 8,
    mtrL_motorPinA1  = 12,
    mtrL_motorPinB1  = 13
};

// Control loop rate.
unsigned long cycleTimeMillis = 100;
unsigned long previousMillis = 0;

volatile long int mtrR_encoderCount = 0;
volatile long int mtrL_encoderCount = 0;

// Construct the motor driver for use.
Spg30MotorDriver rightMotorController(cycleTimeMillis, mtrR_motorPinA1, mtrR_motorPinB1, 
                                  mtrR_motorPwmPin, mtrR_encoderCount);

Spg30MotorDriver leftMotorController(cycleTimeMillis, mtrL_motorPinA1, mtrL_motorPinB1, 
                                  mtrL_motorPwmPin, mtrL_encoderCount);
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

bool mtrL_A_set, mtrL_B_set;
bool mtrR_A_set, mtrR_B_set;

void setup() {                       
    Serial.begin(115600);
    digitalWrite(mtrR_encoderPinA, HIGH);
    digitalWrite(mtrR_encoderPinB, HIGH);
    digitalWrite(mtrL_encoderPinA, HIGH);
    digitalWrite(mtrL_encoderPinB, HIGH);
    // Very Important - attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(mtrR_encoderPinA), ISR_mtrR_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mtrR_encoderPinB), ISR_mtrR_encoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mtrL_encoderPinA), ISR_mtrL_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mtrL_encoderPinB), ISR_mtrL_encoderB, CHANGE);
    TestNumber = FWD_POS_TEST;
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= cycleTimeMillis) {
        /// - Save off current millis. Used for control loop timing.
        previousMillis = currentMillis;
        run_tests();
    }
}

void run_tests(){
    switch(TestNumber)
    {
        case FWD_POS_TEST:
            rightMotorController.ControlMode = Spg30MotorDriver::POSITION; 
            rightMotorController.PositionCmd(720); 
            rightMotorController.run();
            leftMotorController.ControlMode = Spg30MotorDriver::POSITION; 
            leftMotorController.PositionCmd(720); 
            leftMotorController.run();
            if (rightMotorController.ReachedPosition() &&
                leftMotorController.ReachedPosition()) {
                 Serial.println("FWD POSITION TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING BWD POSITION TEST");
                 TestNumber = BWD_POS_TEST;
            }
        break;

        case BWD_POS_TEST:
            rightMotorController.ControlMode = Spg30MotorDriver::POSITION; 
            rightMotorController.PositionCmd(-720); 
            rightMotorController.run();
            leftMotorController.ControlMode = Spg30MotorDriver::POSITION; 
            leftMotorController.PositionCmd(-720); 
            leftMotorController.run();
            if (rightMotorController.ReachedPosition() &&
                leftMotorController.ReachedPosition()) {
                 Serial.println("BWD POSITION TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING FWD VELOCITY TEST");
    //               Uncomment line below when ready to test velocity control                 
    //                 TestNumber = FWD_VEL_TEST;
                 TestNumber = FWD_POS_TEST;
            }
        break;

        case FWD_VEL_TEST:
            rightMotorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            rightMotorController.VelocityCmd(25); 
            rightMotorController.run();
            if (rightMotorController.ReachedVelocity()){
                 Serial.println("FWD VELOCITY TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING ZERO VELOCITY TEST");
                 TestNumber = ZERO_VEL_TEST;
            }
        break;

        case ZERO_VEL_TEST:
            rightMotorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            rightMotorController.VelocityCmd(0); 
            rightMotorController.run();
            if (rightMotorController.ReachedVelocity()){
                 Serial.println("ZERO VELOCITY TEST COMPLETE");
                 delay(5000);
                 Serial.println("STARTING BWD VELOCITY TEST");
                 TestNumber = BWD_VEL_TEST;
            }
        break;

        case BWD_VEL_TEST:
            rightMotorController.ControlMode = Spg30MotorDriver::VELOCITY; 
            rightMotorController.VelocityCmd(-25); 
            rightMotorController.run();
            if (rightMotorController.ReachedVelocity()){
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
}
// Interrupt Right Motor on A changing state
void ISR_mtrR_encoderA(){
    // Test transition
    mtrR_A_set = digitalRead(mtrR_encoderPinA) == HIGH;
    // and adjust counter + if A leads B
    mtrR_encoderCount += (mtrR_A_set != mtrR_B_set) ? +1 : -1;
}

// Interrupt Right Motor on B changing state
void ISR_mtrR_encoderB(){
    // Test transition
    mtrR_B_set = digitalRead(mtrR_encoderPinB) == HIGH;
    // and adjust counter + if B follows A
    mtrR_encoderCount += (mtrR_A_set == mtrR_B_set) ? +1 : -1;
}

// Interrupt Left Motor on A changing state
void ISR_mtrL_encoderA(){
    // Test transition
    mtrL_A_set = digitalRead(mtrL_encoderPinA) == HIGH;
    // and adjust counter + if A leads B
    mtrL_encoderCount += (mtrL_A_set != mtrL_B_set) ? +1 : -1;
}

// Interrupt Left Motor on B changing state
void ISR_mtrL_encoderB(){
    // Test transition
    mtrL_B_set = digitalRead(mtrL_encoderPinB) == HIGH;
    // and adjust counter + if B follows A
    mtrL_encoderCount += (mtrL_A_set == mtrL_B_set) ? +1 : -1;
}
