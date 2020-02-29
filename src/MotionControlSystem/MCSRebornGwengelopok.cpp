//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

#include "MCS.h"


MCS::MCS(): leftMotor(Side::LEFT), rightMotor(Side::RIGHT)  {

#if defined(MAIN)
    encoderLeft = new Encoder(ENCODER_LEFT_B,ENCODER_LEFT_A);
    encoderRight = new Encoder(ENCODER_RIGHT_B,ENCODER_RIGHT_A);
#elif defined(SLAVE)
    encoderLeft = new Encoder(ENCODER_LEFT_A,ENCODER_LEFT_B);
    encoderRight = new Encoder(ENCODER_RIGHT_A,ENCODER_RIGHT_B);
#endif

    initCommunicationBuffers();
    initSettings();
    initStatus();
    // FIXME : ? Duplication de ce que fait initStatus ?
    robotStatus.controlled = true;
    robotStatus.controlledRotation = true;
    robotStatus.controlledTranslation = true;
    robotStatus.inRotationInGoto = false;
    robotStatus.inGoto = false;
    robotStatus.sentMoveAbnormal = false;
    robotStatus.movement = MOVEMENT::NONE;
    expectedWallImpact = false;
    rotationPID.active = false;
    translationPID.active = false;
    robotStatus.translation = false;


#if defined(MAIN)

    leftSpeedPID.setTunings(1, 0.00, 0, 0); //0.5   0.000755   21.5
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(1.1, 0.00, 0, 0); //0.85 0.000755 0
    rightSpeedPID.enableAWU(false);

    translationPID.setTunings(2,0,0,0);
    translationPID.enableAWU(false);
    rotationPID.setTunings(2.2,0,0,0);
    rotationPID.enableAWU(false);

#elif defined(SLAVE)

/* asserv en vitesse */
    leftSpeedPID.setTunings(0.53, 0.00105, 30, 0);//0.0015
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.718591667, 0.00125, 30, 0);//0.0015
    rightSpeedPID.enableAWU(false);
/* asserv en translation */
    translationPID.setTunings(2.75,0,5,0);//2.75  0  5
    translationPID.enableAWU(false);
/* asserv en rotation */
    rotationPID.setTunings(3.2,0.0000,0,0);  //3.2  0  0
    rotationPID.enableAWU(false);

#endif

    leftMotor.init();
    rightMotor.init();
}

void MCS::initCommunicationBuffers() {
    singleBoolArgBuffer = new I2CC::BufferedData(sizeof(bool)*1);
    singleFloatArgBuffer = new I2CC::BufferedData(sizeof(float)*1);
    singleInt16ArgBuffer = new I2CC::BufferedData(sizeof(int16_t)*1);
    gotoArgBuffer = new I2CC::BufferedData(sizeof(float)*3);
    setXYOArgBuffer = new I2CC::BufferedData(sizeof(int16_t)*2);
    returnDataTicks = new I2CC::BufferedData(sizeof(int32_t)*2);
    returnRawPosDataBuffer = new I2CC::BufferedData(sizeof(int16_t)*2+ sizeof(float)*3+ sizeof(long)*2);
}

void MCS::initSettings() {
    robotStatus.inRotationInGoto = false;
    robotStatus.movement = MOVEMENT::NONE;


    /* mm/s/MCS_PERIOD */
    controlSettings.maxAcceleration = 1.5;//2;
    controlSettings.maxDeceleration = 2;//2;

    /* rad/s */
    controlSettings.maxRotationSpeed = 2*PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 1000;
    controlSettings.tolerancySpeed = 5;

    /* rad */
#if defined(MAIN)
    controlSettings.tolerancyAngle = 0.0005;
#elif defined(SLAVE)
    controlSettings.tolerancyAngle = 0.01;
#endif

    /* mm */
#if defined(MAIN)
    controlSettings.tolerancyTranslation = 1;
    controlSettings.tolerancyX=10;
    controlSettings.tolerancyY=10;
#elif defined(SLAVE)
    controlSettings.tolerancyTranslation = 1;
    controlSettings.tolerancyX=10;
    controlSettings.tolerancyY=10;
#endif

    /* ms */
    controlSettings.stopDelay = 25;

    /* mm/s */
#if defined(MAIN)
    controlSettings.tolerancyDerivative = 7;
#elif defined(SLAVE)
    controlSettings.tolerancyDerivative = 0.0000001; //à laisser très petit sinon le robot ne s'arrete pas
#endif

    /* patate */
    controlSettings.tolerancyDifferenceSpeed = 500*2;
}

void MCS::initStatus() {
    robotStatus.movement = MOVEMENT::NONE;
    robotStatus.moving = false;
    robotStatus.inRotationInGoto = false;
    robotStatus.inGoto = false;
    robotStatus.controlled = true;
    robotStatus.controlledRotation = true;
    robotStatus.controlledTranslation = true;
    previousLeftSpeedGoal = 0;
    previousRightSpeedGoal = 0;
    previousLeftTicks = 0;
    previousRightTicks = 0;
}

void MCS::stop() {
    I2CC::executeRPC(MCS_SLAVE_ID, STOP_RPC_ID);
}

void MCS::translate(int16_t amount) {
    singleInt16ArgBuffer->rewind();
    I2CC::putData<int16_t>(amount, singleInt16ArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, TRANSLATE_RPC_ID, singleInt16ArgBuffer);
}

void MCS::rotate(float angle) {
    singleFloatArgBuffer->rewind();
    I2CC::putData<float>(angle, singleFloatArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, ROTATE_RPC_ID, singleFloatArgBuffer);
}

void MCS::gotoPoint(int16_t x, int16_t y) {
    gotoArgBuffer->rewind();
    I2CC::putData(x, gotoArgBuffer);
    I2CC::putData(y, gotoArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, GOTO_RPC_ID, gotoArgBuffer);
}

void MCS::setXYO(int16_t x, int16_t y, float angle) {
    setXYOArgBuffer->rewind();
    I2CC::putData(x, setXYOArgBuffer);
    I2CC::putData(y, setXYOArgBuffer);
    I2CC::putData(angle, setXYOArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, SET_XYO_RPC_ID, setXYOArgBuffer);
}

void MCS::stopTranslation() {
    robotStatus.speedTranslation = 0.0f;
}

void MCS::stopRotation() {
    robotStatus.speedRotation = 0.0f;
}

void MCS::speedBasedMovement(MOVEMENT movement) {
    if(!robotStatus.controlled)
    {
        return;
    }

    robotStatus.moving = true;

    switch(movement)
    {
        case MOVEMENT::FORWARD:
            I2CC::executeRPC(MCS_SLAVE_ID, FORCE_FORWARD_RPC_ID);
            break;

        case MOVEMENT::BACKWARD:
            I2CC::executeRPC(MCS_SLAVE_ID, FORCE_BACKWARDS_RPC_ID);
            break;

        case MOVEMENT::TRIGO:
            I2CC::executeRPC(MCS_SLAVE_ID, FORCE_TURN_LEFT_RPC_ID);
            break;

        case MOVEMENT::ANTITRIGO:
            I2CC::executeRPC(MCS_SLAVE_ID, FORCE_TURN_RIGHT_RPC_ID);
            break;

        case MOVEMENT::NONE:
        default:
            I2CC::executeRPC(MCS_SLAVE_ID, FORCE_STOP_RPC_ID);
            return;
    }
    robotStatus.movement = movement;
}

void MCS::sendPositionUpdate() {
    returnPosUpdateBuffer->rewind();
    I2CC::dataRequest(MCS_SLAVE_ID, POS_UPDATE_RPC_ID, *returnPosUpdateBuffer, nullptr);
    I2CC::getData(robotStatus.x, returnPosUpdateBuffer);
    I2CC::getData(robotStatus.y, returnPosUpdateBuffer);
    I2CC::getData(robotStatus.orientation, returnPosUpdateBuffer);
    // FIXME : Does not seem to work properly
    ComMgr::Instance().printfln(POSITION_HEADER, "%f %f %f %li", robotStatus.x, robotStatus.y, robotStatus.orientation, millis());
}

void MCS::resetEncoders() {
    leftTicks = 0;
    rightTicks = 0;
    encoderLeft->write(0);
    encoderRight->write(0);
    previousLeftTicks = 0;
    previousRightTicks = 0;
    currentDistance = 0;
    translationPID.setGoal(currentDistance);
    rotationPID.setGoal(robotStatus.orientation);
}

void MCS::disableP2P() {
    trajectory.clear();
    robotStatus.inRotationInGoto = false;
}

void MCS::setControl(bool b) {
    singleBoolArgBuffer->rewind();
    I2CC::putData(b, singleBoolArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, CHANGE_CONTROL_STATE_RPC_ID, singleBoolArgBuffer);

    robotStatus.controlled = b;
}

void MCS::controlledTranslation(bool b) {
    singleBoolArgBuffer->rewind();
    I2CC::putData(b, singleBoolArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, CHANGE_TRANSLATION_CONTROL_STATE_RPC_ID, singleBoolArgBuffer);

    robotStatus.controlledTranslation = b;
}

void MCS::controlledRotation(bool b) {
    singleBoolArgBuffer->rewind();
    I2CC::putData(b, singleBoolArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, CHANGE_ROTATION_CONTROL_STATE_RPC_ID, singleBoolArgBuffer);

    robotStatus.controlledRotation = b;
}

void MCS::setForcedMovement(bool newState) {
    singleBoolArgBuffer->rewind();
    I2CC::putData(newState, singleBoolArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, CHANGE_FORCED_MOVEMENT_STATE_RPC_ID, singleBoolArgBuffer);

    robotStatus.forcedMovement = newState;
}

void MCS::setTranslationSpeed(float speed) {
    robotStatus.speedTranslation = speed;
}

void MCS::setRotationSpeed(float speed) {
    robotStatus.speedRotation = speed;
}

void MCS::setMaxTranslationSpeed(float speed) {
    singleFloatArgBuffer->rewind();
    I2CC::putData(speed, singleFloatArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, SET_TRANSLATION_SPEED_RPC_ID, singleFloatArgBuffer);

    controlSettings.maxTranslationSpeed = speed;
}

void MCS::setMaxRotationSpeed(float speed) {
    singleFloatArgBuffer->rewind();
    I2CC::putData(speed, singleFloatArgBuffer);
    I2CC::executeRPC(MCS_SLAVE_ID, SET_ROTATION_SPEED_RPC_ID, singleFloatArgBuffer);

    controlSettings.maxRotationSpeed = speed;
}

int16_t MCS::getX() {
    return (int16_t) robotStatus.x;
}

int16_t MCS::getY() {
    return (int16_t) robotStatus.y;
}

float MCS::getAngle() {
    return robotStatus.orientation;
}

void MCS::setX(int16_t x) {
    robotStatus.x = x;
}

void MCS::setY(int16_t y) {
    robotStatus.y = y;
}

void MCS::setAngle(float angle) {
    robotStatus.orientation = angle;
}

void MCS::setAngleOffset(float offset) {
    angleOffset = offset;
}

void MCS::getTicks(int32_t& left, int32_t& right) {
    returnDataTicks->rewind();
    I2CC::dataRequest(MCS_SLAVE_ID, GET_TICKS_RPC_ID, *returnDataTicks, nullptr);
    I2CC::getData(left, returnDataTicks);
    I2CC::getData(right, returnDataTicks);
}

float MCS::getLeftSpeed() {
    return robotStatus.speedLeftWheel;
}

float MCS::getRightSpeed() {
    return robotStatus.speedRightWheel;
}

void MCS::getSpeedGoals(long &leftGoal, long &rightGoal) {
    leftGoal = robotStatus.leftSpeedGoal;
    rightGoal = robotStatus.rightSpeedGoal;
}

void MCS::expectWallImpact()
{
    expectedWallImpact = true;
}

bool MCS::sentMoveAbnormal() {
    return robotStatus.sentMoveAbnormal;
}

bool MCS::isMoveAbnormal() {
    return robotStatus.stuck;
}

void MCS::setMoveAbnormalSent(bool val) {
    robotStatus.sentMoveAbnormal = val;
}

void MCS::queryRawPosData() {
    returnRawPosDataBuffer->rewind();
    I2CC::dataRequest(MCS_SLAVE_ID, GET_RAW_POS_DATA_RPC_ID, *returnRawPosDataBuffer, nullptr);

    I2CC::getData(robotStatus.x, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.y, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.orientation, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.speedLeftWheel, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.leftSpeedGoal, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.speedRightWheel, returnRawPosDataBuffer);
    I2CC::getData(robotStatus.rightSpeedGoal, returnRawPosDataBuffer);
}