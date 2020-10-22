// ActuatorsMgr.h


#ifndef _ACTUATORSMGR_h
#define _ACTUATORSMGR_h

#include "Arduino.h"


#include "DynamixelManager.h"
#include "XL430.h"
#include "SyncWrite.h"

#include "Config/Defines.h"
#include "Config/PinMapping.h"
#include "Utils/Singleton.hpp"
#include "Utils/Utils.h"
#include <Stepper.h>
#include "Servo.h"

#include "ActuatorValues.h"
#include "Arm.hpp"
#include <vector>

static HardwareSerial& XLSerial = Serial1;

enum StepperDirection {
	UP, DOWN
};

class ActuatorsMgr : public Singleton<ActuatorsMgr>
{
public:
    void init() override;

    //Gestion des XL430
    DynamixelManager* dynamixelManager;

    // TODO : Généraliser
    // Liste des moteurs du bras 1


    Arm<XL430>* rightArm;
    Servo* motFlag;

#if defined(MAIN)

    // Liste des moteurs du bras 2
    XL430* motor0;
    XL430* motor1;
    XL430* motor2;
	XL430* motor3;
    XL430* motor4;
	XL430* motor5;
    XL430* motor6;
    XL430* motor7;
    XL430* motor8;
    XL430* motor9;
    XL430* motor10;

	Arm<XL430>* leftArm;
    Servo* motRight;
    Servo* motLeft;

    // Stepper pour l'ascenseur porte-gobelets
    Stepper* stepper;


#elif defined(SLAVE)

    //XL qui pousse les palets
    XL430* motor0;
    XL430* motor1;
    XL430* motor2;
	XL430* motor3;
	XL430* motor4;
	XL430* motor5;
    XL430* motor6;
    XL430* motor7;
    XL430* motor8;
    XL430* motor9;
    XL430* motor10;

#endif

	ActuatorsMgr() = default;

	/**
    * Appelé tous les 800 µs pour faire bouger les steppers
    */
	void handleInterrupt();
	void initTorques();
	void moveLeftStepper(int32_t count, int32_t nextCount = 0);
	void moveRightStepper(int32_t count, int32_t nextCount = 0);
	void moveRightStepperOust(int32_t count, int32_t nextCount = 0);
	void moveRightStepperOust2(int32_t count, int32_t nextCount = 0);

	void checkArmMovements();
	void rebootArms();


private:
    StepperDirection leftDirection;
    StepperDirection rightDirection;
    volatile int32_t leftStepCount;
    volatile int32_t rightStepCount;
    int32_t nextLeftStepCount;
    int32_t nextRightStepCount;
    volatile int32_t timerForLeftStepper = -1;
    volatile int32_t timerForRightStepper = -1;

};

void stepperInterrupt(HardwareTimer*);
#endif