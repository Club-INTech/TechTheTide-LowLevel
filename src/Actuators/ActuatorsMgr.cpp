// 
// 
// 

#include <COM/InterruptStackPrint.h>
#include "ActuatorsMgr.h"

void ActuatorsMgr::init()
{
    dynamixelManager = new DynamixelManager(PIN_RX_DYNAMIXEL,PIN_TX_DYNAMIXEL, &DebugSerial);

    motor0 = (XL430*) dynamixelManager->createMotor(0, XL430GeneratorFunction);//new XL430(1,*manager);
    motor1 = (XL430*) dynamixelManager->createMotor(1, XL430GeneratorFunction);//new XL430(1,*manager);
    motor2 = (XL430*) dynamixelManager->createMotor(2, XL430GeneratorFunction);//new XL430(2,*manager);
    motor3 = (XL430*) dynamixelManager->createMotor(3, XL430GeneratorFunction);//new XL430(3,*manager);
    motor4 = (XL430*) dynamixelManager->createMotor(4, XL430GeneratorFunction);//new XL430(4,*manager);
    motor5 = (XL430*) dynamixelManager->createMotor(5, XL430GeneratorFunction);//new XL430(5,*manager);
    motor6 = (XL430*) dynamixelManager->createMotor(6, XL430GeneratorFunction);//new XL430(1,*manager);
    motor7 = (XL430*) dynamixelManager->createMotor(7, XL430GeneratorFunction);//new XL430(1,*manager);
    motor8 = (XL430*) dynamixelManager->createMotor(8, XL430GeneratorFunction);//new XL430(2,*manager);
    motor9 = (XL430*) dynamixelManager->createMotor(9, XL430GeneratorFunction);//new XL430(3,*manager);
    motor10 = (XL430*) dynamixelManager->createMotor(10, XL430GeneratorFunction);//new XL430(4,*manager);

    rightArm = new Arm<XL430>("right", *dynamixelManager, new XL430[3]{*motor1, *motor2, *motor3});



#if defined(MAIN)

//    motor0 = (XL430*) dynamixelManager->createMotor(0, XL430GeneratorFunction);//new XL430(1,*manager);
//    motor1 = (XL430*) dynamixelManager->createMotor(1, XL430GeneratorFunction);//new XL430(1,*manager);
//    motor2 = (XL430*) dynamixelManager->createMotor(2, XL430GeneratorFunction);//new XL430(2,*manager);
//    motor3 = (XL430*) dynamixelManager->createMotor(3, XL430GeneratorFunction);//new XL430(3,*manager);
//    motor4 = (XL430*) dynamixelManager->createMotor(4, XL430GeneratorFunction);//new XL430(4,*manager);
//    motor5 = (XL430*) dynamixelManager->createMotor(5, XL430GeneratorFunction);//new XL430(5,*manager);
//
    leftArm = new Arm<XL430>("left", *dynamixelManager, new XL430[3]{*motor1,*motor2,*motor3});

    motRight->attach(MG_COULOIR_D);
    motLeft->attach(MG_COULOIR_G);
    motFlag->attach(FLAG_PIN,1000,2000);

    stepper = new Stepper(50, 3, 4, 5, 6);  //A CHECKER DANS LA DATASHEEEEEEEET, PARCE QUE 50 CA PUE DU CUL
    stepper->setSpeed(60);

#elif defined(SLAVE)
    motFlag = new Servo;
    motFlag->attach(FLAG_PIN);

#endif
}

void stepperInterrupt(HardwareTimer* hardwareTimer) {
    ActuatorsMgr::Instance().handleInterrupt();
}

void ActuatorsMgr::initTorques() {
    motor0->toggleTorque(true);
    motor1->toggleTorque(true);
    motor2->toggleTorque(true);
    motor3->toggleTorque(true);
    motor4->toggleTorque(true);
    motor5->toggleTorque(true);
    motor6->toggleTorque(true);
    motor7->toggleTorque(true);
    motor8->toggleTorque(true);
    motor9->toggleTorque(true);
    motor10->toggleTorque(true);
// FIXME : À compléter pour utiliser correctement les moteurs
}


void ActuatorsMgr::handleInterrupt() {
#if defined(MAIN)
    if(leftStepCount > 0) {
        if(leftStepCount % 2 == 0) {
            digitalWrite(STEP_PIN_LEFT, HIGH);
        } else {
            digitalWrite(STEP_PIN_LEFT, LOW);
        }
        leftStepCount--;

        if(leftStepCount == 0 && nextLeftStepCount == 0) {
            InterruptStackPrint::Instance().push(EVENT_HEADER, "leftElevatorStopped");
        }
    } else {
        leftStepCount = 0;
        digitalWrite(STEP_PIN_LEFT, LOW);

        if(timerForLeftStepper < 0) {
            timerForLeftStepper = STEPPER_DELAY;
        }
        if(timerForLeftStepper > 0) {
            timerForLeftStepper--;
        }
        if(timerForLeftStepper == 0) {
            moveLeftStepper(nextLeftStepCount);
            nextLeftStepCount = 0;
        }
    }
#endif

    if(rightStepCount > 0) {
        if (rightStepCount % 2 == 0) {
            digitalWrite(STEP_PIN_RIGHT, HIGH);
        } else {
            digitalWrite(STEP_PIN_RIGHT, LOW);
        }
        rightStepCount--;

        if(rightStepCount == 0 && nextRightStepCount == 0) {
            InterruptStackPrint::Instance().push(EVENT_HEADER, "rightElevatorStopped");
        }
    } else {
        rightStepCount = 0;
        digitalWrite(STEP_PIN_RIGHT, LOW);

        if(timerForRightStepper < 0) { // lance l'attente
            timerForRightStepper = STEPPER_DELAY;
        }
        if(timerForRightStepper > 0) {
            timerForRightStepper--;
        }
        if(timerForRightStepper == 0) { // si l'attente est finie
            moveRightStepper(nextRightStepCount);
            nextRightStepCount = 0;
        }
    }
}

#if defined(MAIN)
void ActuatorsMgr::moveLeftStepper(int32_t count, int32_t nextCount) {
    timerForLeftStepper = STEPPER_DELAY;
    this->leftDirection = count > 0 ? UP : DOWN;

    // inversé par rapport à droite
    if(count > 0) {
        digitalWrite(DIR_PIN_LEFT, HIGH);
    } else {
        digitalWrite(DIR_PIN_LEFT, LOW);
    }
    leftStepCount = ABS(count)*STEP_COUNT;

    nextLeftStepCount = nextCount;
}
#endif

void ActuatorsMgr::moveRightStepper(int32_t count, int32_t nextCount) {
    timerForRightStepper = STEPPER_DELAY;
    this->rightDirection = count > 0 ? UP : DOWN;
    if(count > 0) {
        digitalWrite(DIR_PIN_RIGHT, HIGH);
    } else {
        digitalWrite(DIR_PIN_RIGHT, LOW);
    }
    rightStepCount = ABS(count)*STEP_COUNT;

    nextRightStepCount = nextCount;
}

#if defined(SLAVE)

void ActuatorsMgr::moveRightStepperOust(int32_t count, int32_t nextCount) {
    timerForRightStepper = STEPPER_DELAY;
    this->rightDirection = count > 0 ? UP : DOWN;
    if(count > 0) {
        digitalWrite(DIR_PIN_RIGHT, HIGH);
    } else {
        digitalWrite(DIR_PIN_RIGHT, LOW);
    }
    rightStepCount = ABS(count)*STEP_COUNT_OUST;

    nextRightStepCount = nextCount;
    //   analogWrite(STEP_PIN_RIGHT, 128);
/*    rightStepCount += count*STEP_COUNT_OUST;
    rightStepper.setTargetAbs(rightStepCount);
    stepControl.moveAsync(rightStepper);*/
}

void ActuatorsMgr::moveRightStepperOust2(int32_t count, int32_t nextCount) {
    timerForRightStepper = STEPPER_DELAY;
    this->rightDirection = count > 0 ? UP : DOWN;
    if(count > 0) {
        digitalWrite(DIR_PIN_RIGHT, HIGH);
    } else {
        digitalWrite(DIR_PIN_RIGHT, LOW);
    }
    rightStepCount = ABS(count)*(STEP_COUNT+STEP_COUNT_OUST);

    nextRightStepCount = nextCount;
    //   analogWrite(STEP_PIN_RIGHT, 128);
/*    rightStepCount += count*STEP_COUNT_OUST;
    rightStepper.setTargetAbs(rightStepCount);
    stepControl.moveAsync(rightStepper);*/
}

#endif

void ActuatorsMgr::checkArmMovements() {
#if defined(MAIN)
    leftArm->update();
#endif
    rightArm->update();
}

void ActuatorsMgr::rebootArms() {
#if defined(MAIN)
    leftArm->rebootXLsIfNecessary();
#endif
    rightArm->rebootXLsIfNecessary();
}