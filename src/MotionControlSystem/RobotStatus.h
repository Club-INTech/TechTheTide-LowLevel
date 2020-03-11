//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_ROBOTSTATUS_H
#define LL_ROBOTSTATUS_H

#include <cstdint>

enum class MOVEMENT { FORWARD, BACKWARD, TRIGO, ANTITRIGO, CURVE, NONE };

enum MovementStatus : uint8_t {
    CURRENTLY_MOVING = 0,
    STOPPED_MOVING,
    UNABLE_TO_MOVE,
    RETRY_GOTO,
};

struct RobotStatus
{
    bool translation;
    bool controlled;
    bool controlledTranslation;
    bool controlledRotation;
    bool inRotationInGoto;
    bool inGoto;
    bool forcedMovement;
    bool moving;
    MovementStatus notMoving;
    bool stuck;

    float x;
    float y;
    float orientation;

    MOVEMENT movement;

    float speedTranslation;
    float speedRotation;
    float speedLeftWheel;
    float speedRightWheel;

    long leftSpeedGoal;
    long rightSpeedGoal;

    bool sentMoveAbnormal;

    uint64_t controlBoardTime;

    RobotStatus();
    void updateStatus();
};

#endif //LL_ROBOTSTATUS_H
