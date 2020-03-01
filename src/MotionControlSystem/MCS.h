//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_MCSREBORN_H
#define LL_MCSREBORN_H

#include "Utils/Singleton.hpp"
#include "Utils/Average.hpp"
#include "Utils/Utils.h"
#include "Config/Defines.h"
#include "Config/PinMapping.h"
#include "COM/ComMgr.h"
#include "COM/InterruptStackPrint.h"

#include "ControlSettings.h"
#include "RobotStatus.h"
#include "Motor.h"
#include "PID.hpp"
#include "SelfContainedPID.hpp"
#include "PointToPointTrajectory.h"
#include <I2CC.h>
#include "COM/SlaveIDs.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"

#include <cmath>

// RPC ids used by the MCS
constexpr uint8_t GOTO_RPC_ID = 1;
constexpr uint8_t STOP_RPC_ID = 2;
constexpr uint8_t POS_UPDATE_RPC_ID = 3;
constexpr uint8_t TRANSLATE_RPC_ID = 4;
constexpr uint8_t ROTATE_RPC_ID = 5;
constexpr uint8_t SET_XYO_RPC_ID = 6;
constexpr uint8_t SET_TRANSLATION_SPEED_RPC_ID = 7;
constexpr uint8_t SET_ROTATION_SPEED_RPC_ID = 8;
constexpr uint8_t CHANGE_TRANSLATION_CONTROL_STATE_RPC_ID = 9;
constexpr uint8_t CHANGE_ROTATION_CONTROL_STATE_RPC_ID = 10;
constexpr uint8_t CHANGE_FORCED_MOVEMENT_STATE_RPC_ID = 11;
constexpr uint8_t FORCE_FORWARD_RPC_ID = 12;
constexpr uint8_t FORCE_BACKWARDS_RPC_ID = 13;
constexpr uint8_t FORCE_TURN_LEFT_RPC_ID = 14;
constexpr uint8_t FORCE_TURN_RIGHT_RPC_ID = 15;
constexpr uint8_t CHANGE_CONTROL_STATE_RPC_ID = 16;
constexpr uint8_t GET_RAW_POS_DATA_RPC_ID = 17;
constexpr uint8_t GET_TICKS_RPC_ID = 18;
constexpr uint8_t FORCE_STOP_RPC_ID = 19;
constexpr uint8_t GET_XYO_RPC_ID = 21;

// TODO : Tout docu
// TODO : P'tet passer les config dans un fichier dans src/Config ?
class MCS : public Singleton<MCS>
{
public:
    MCS();

    void initEncoders();

    void stop();
    void stopTranslation();
    void stopRotation();

    void translate(int16_t);
    void rotate(float);
    void gotoPoint(int16_t,int16_t);
    void followTrajectory(const double* xTable, const double* yTable, int count);

    void speedBasedMovement(MOVEMENT);

    void setControl(bool);
    void controlledTranslation(bool);
    void controlledRotation(bool);
    void setForcedMovement(bool);
    void setTranslationSpeed(float);
    void setRotationSpeed(float);
    void setMaxTranslationSpeed(float);
    void setMaxRotationSpeed(float);

    void initSettings();
    void initStatus();

    /**
     * Initialise les différents BufferedData utilisés pour communiquer avec la carte Asserv'.
     * Evite de faire des 'new' à chaque fois qu'on a besoin d'un buffer
     */
    void initCommunicationBuffers();

    /**
     * Méthode appelée par un InterruptTimer afin d'envoyer au HL la position du robot
     */
    void sendPositionUpdate();

    /**
     * Reset des codeuses, utilisé quand le HL reset la position du robot (grâce aux SICK par exemple)
     */
    void resetEncoders();

    int16_t getX();
    int16_t getY();
    float getAngle();
    void getTicks(int32_t& left, int32_t& right);
    float getLeftSpeed();
    float getRightSpeed();

    void getSpeedGoals(long&,long&);

    /**
     * Permet de définir une rotation à la fin d'un mouvement (au lieu de devoir attendre la fin du mouvement et de donner un ordre de rotation)
     * /!\\ Cette valeur est réinitialisée dès la fin du mouvement!!! (Histoire de pas se décaler avec les mouvements suivants)
     * @param offset l'angle, en radians, duquel le robot doit tourner à la fin du mouvement
     */
    void setAngleOffset(float offset);

    /**
     * Annule le suivi de trajectoire courant
     */
    void disableP2P();

    void setX(int16_t);
    void setY(int16_t);
    void setAngle(float);

    void setXYO(int16_t, int16_t, float);

    void expectWallImpact();

    bool sentMoveAbnormal();
    bool isMoveAbnormal();
    void setMoveAbnormalSent(bool);

    void queryRawPosData();
    void queryXYO();


private:

    Encoder* encoderRight = nullptr;
    Encoder* encoderLeft = nullptr;

    RobotStatus robotStatus;
    ControlSettings controlSettings;

    Motor leftMotor;
    Motor rightMotor;

    SelfContainedPID<float> leftSpeedPID;
    SelfContainedPID<float> rightSpeedPID;
    SelfContainedPID<float> translationPID;
//    SelfContainedPID<float> rotationPID180;
//    SelfContainedPID<float> rotationPID90;
    SelfContainedPID<float> rotationPID;

    int32_t currentDistance;
    int16_t targetX;
    int16_t targetY;

    int32_t leftTicks;
    int32_t rightTicks;
    int32_t previousLeftTicks;
    int32_t previousRightTicks;
    float previousLeftSpeedGoal;
    float previousRightSpeedGoal;
    int16_t targetDistance;
    float targetAngle;
    float angleOffset;
    bool expectedWallImpact;

    Average<float, 100> averageLeftSpeed;
    Average<float, 100> averageRightSpeed;
#if defined(MAIN)
    Average<float, 25> averageRotationDerivativeError;
    Average<float, 25> averageTranslationDerivativeError;
#elif defined(SLAVE)
    Average<float, 10> averageRotationDerivativeError;
    Average<float, 10> averageTranslationDerivativeError;
#endif

    bool sequentialMovement;
    PointToPointTrajectory trajectory;

    // Timer entre translation et rotation pour les goto
    uint32_t gotoTimer;

    // read buffers
    I2CC::BufferedData* returnDataTicks;
    I2CC::BufferedData* returnRawPosDataBuffer;
    I2CC::BufferedData* returnPosUpdateBuffer;
    I2CC::BufferedData* returnGotoBuffer;
    I2CC::BufferedData* returnXYO;

    // write buffers
    I2CC::BufferedData* singleBoolArgBuffer;
    I2CC::BufferedData* singleFloatArgBuffer;
    I2CC::BufferedData* singleInt16ArgBuffer;
    I2CC::BufferedData* gotoArgBuffer;
    I2CC::BufferedData* setXYOArgBuffer;
};

#endif //LL_MCSREBORN_H
