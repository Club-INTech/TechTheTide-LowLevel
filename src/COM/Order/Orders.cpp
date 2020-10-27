//
// Created by asphox on 29/04/18.
//

#include "Orders.h"



// TODO : Nettoyer


using namespace I2CC;


void ORDER_ping::impl(Args args)
{
    orderManager.highLevel.printfln(EVENT_HEADER,"pong");

    I2CC::BufferedData returnData(4*sizeof(char));
    ((char*)returnData.dataArray)[0] = 'h';
    ((char*)returnData.dataArray)[1] = 'o';
    ((char*)returnData.dataArray)[2] = 'w';
    ((char*)returnData.dataArray)[3] = '?';
    bool valid = I2CC::dataRequest(MCS_SLAVE_ID, 0, returnData, nullptr);
    char buf[5];
    buf[0] = ((char*)returnData.dataArray)[0];
    buf[1] = ((char*)returnData.dataArray)[1];
    buf[2] = ((char*)returnData.dataArray)[2];
    buf[3] = ((char*)returnData.dataArray)[3];
    buf[4] = '\0';

    Serial.println(buf);
    Serial.println(valid ? "ok" : "NOK");
}

void ORDER_j::impl(Args args)
{
    orderManager.HLWaiting = true;
}

void ORDER_f::impl(Args args)
{
    /* FIXME orderManager.highLevel.printfln(STD_HEADER,"%d",orderManager.motionControlSystem.isMoving());
    orderManager.highLevel.printfln(STD_HEADER,"%d",orderManager.motionControlSystem.isMoveAbnormal());*/
}

void ORDER_xyo::impl(Args args)
{
    orderManager.motionControlSystem.queryXYO();
    orderManager.highLevel.printfln(STD_HEADER,"%i",orderManager.motionControlSystem.getX());
    orderManager.highLevel.printfln(STD_HEADER,"%i",orderManager.motionControlSystem.getY());
    orderManager.highLevel.printfln(STD_HEADER,"%f",orderManager.motionControlSystem.getAngle());
}

void ORDER_mcsTime::impl(Args args) {
    orderManager.motionControlSystem.queryBoardTime();
    orderManager.highLevel.printfln(STD_HEADER,"%i",((int32_t)orderManager.motionControlSystem.getControlBoardTimeMillis()));
    orderManager.highLevel.printfln(STD_HEADER,"%lu",((int32_t)orderManager.motionControlSystem.getControlBoardTimeMicros()));
}

void ORDER_d::impl(Args args)
{
    int16_t deplacement = OrderManager::parseInt(args[0]);
    bool expectedWallImpact = false;
    if(args.nbrParams() == 2) {
        expectedWallImpact = ! strcmp(args[1], "true");
    }
    orderManager.highLevel.printfln(DEBUG_HEADER,"distance : %d %i",deplacement, expectedWallImpact);
    orderManager.motionControlSystem.disableP2P();
    if(expectedWallImpact) {
        orderManager.motionControlSystem.expectWallImpact();
    }
    orderManager.motionControlSystem.translate(deplacement);
}

void ORDER_t::impl(Args args)
{
    float angle;
    if (!strcmp(args[0], "pi")) {
        angle = (float) PI;
    } else {
        angle = OrderManager::parseFloat(args[0]);
    }
    orderManager.highLevel.printfln(DEBUG_HEADER,"angle : %f", angle);

    orderManager.motionControlSystem.disableP2P();
    orderManager.motionControlSystem.rotate(angle);
}

void ORDER_goto::impl(Args args)
{
    float targetX = OrderManager::parseFloat(args[0]);
    float targetY = OrderManager::parseFloat(args[1]);
    bool isSequential = false;

    orderManager.motionControlSystem.gotoPoint(targetX,targetY);
    orderManager.highLevel.printfln(DEBUG_HEADER, "goto %f %f %i", targetX, targetY, isSequential);
}

void ORDER_followTrajectory::impl(Args args)
{
    if(OrderManager::parseFloat(args[0]) == 0)
    {
        // FIXME orderManager.motionControlSystem.followTrajectory(trajectory_S_path[0],trajectory_S_path[1],trajectory_S_size);
    }
    else
    {
        orderManager.highLevel.printfln(DEBUG_HEADER,"ERREUR::Paramètres incorrects");
    }
}

void ORDER_stop::impl(Args args)
{
    orderManager.motionControlSystem.stop();
    orderManager.highLevel.printfln(DEBUG_HEADER,"A priori, je m'arrête");
}

void ORDER_cx::impl(Args args)
{
    orderManager.motionControlSystem.setX(OrderManager::parseFloat(args[0]));
}

void ORDER_cy::impl(Args args)
{
    orderManager.motionControlSystem.setY(OrderManager::parseFloat(args[0]));
}

void ORDER_co::impl(Args args)
{
    orderManager.motionControlSystem.setAngle(OrderManager::parseFloat(args[0]));
}

void ORDER_cxyo::impl(Args args)
{
    orderManager.motionControlSystem.setXYO(OrderManager::parseFloat(args[0]), OrderManager::parseFloat(args[1]), OrderManager::parseFloat(args[2]));
    orderManager.highLevel.printfln(DEBUG_HEADER, "X,Y,O set");
}

void ORDER_ctv::impl(Args args)
{
    orderManager.motionControlSystem.setTranslationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_crv::impl(Args args)
{
    orderManager.motionControlSystem.setRotationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_ctrv::impl(Args args)
{
    orderManager.motionControlSystem.setTranslationSpeed(OrderManager::parseFloat(args[0]));
    orderManager.motionControlSystem.setRotationSpeed(OrderManager::parseFloat(args[1]));
}

void ORDER_efm::impl(Args args)
{
    orderManager.motionControlSystem.setForcedMovement(true);
}

void ORDER_dfm::impl(Args args)
{
    orderManager.motionControlSystem.setForcedMovement(false);
}

void ORDER_ct0::impl(Args args)
{
    orderManager.motionControlSystem.controlledTranslation(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en translation");
}

void ORDER_ct1::impl(Args args)
{
    orderManager.motionControlSystem.controlledTranslation(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Asservi en translation");
}

void ORDER_cr0::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en rotation");
}

void ORDER_cr1::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Asservi en rotation");
}

void ORDER_cv0::impl(Args args)
{
    orderManager.motionControlSystem.setControl(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en vitesse");
}

void ORDER_cv1::impl(Args args)
{
    orderManager.motionControlSystem.setControl(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"asservi en vitesse");
}

void ORDER_cod::impl(Args args)
{
    int32_t left = 42;
    int32_t right = 0;
    orderManager.motionControlSystem.getTicks(left, right);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Gauche: %ld, Droite: %ld", left, right);
}

void ORDER_rawposdata::impl(Args args)
{
    orderManager.motionControlSystem.queryRawPosData();


    int32_t leftSpeedGoal, rightSpeedGoal;
    orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

    int16_t xPos = orderManager.motionControlSystem.getX();
    int16_t yPos = orderManager.motionControlSystem.getY();
    float angle = orderManager.motionControlSystem.getAngle();
    float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
    float rightSpeed = orderManager.motionControlSystem.getRightSpeed();

    //char s[50];
/*
    Serial.print(xPos);
    Serial.print(",");
    Serial.print(yPos);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(leftSpeedGoal);
    Serial.print(",");
    Serial.print(rightSpeed);
    Serial.print(",");
    Serial.print(rightSpeedGoal);
    Serial.println();
*/
    Serial.printf("%d,%d,%f,%f,%ld,%f,%ld\n", xPos,yPos,angle,leftSpeed, leftSpeedGoal,rightSpeed,rightSpeedGoal);
    //int32_t right, left;
    //motionControlSystem.getPWMS(left,right);
    //Serial.println(right);
    //float rotaProp, rotaDer, rotaInt;
    //motionControlSystem.getRotationErrors(rotaProp, rotaInt, rotaDer);
    //Serial.println(rotaInt);
}

void ORDER_reseteth::impl(Args args)
{
    orderManager.highLevel.resetEth();
}

void ORDER_montlhery::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(false);
    orderManager.motionControlSystem.controlledTranslation(false);
    orderManager.motionControlSystem.setForcedMovement(true);
    orderManager.highLevel.printfln(DEBUG_HEADER, "monthlery received");
}

void ORDER_maxtr::impl(Args args) {
    orderManager.motionControlSystem.setMaxTranslationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_maxro::impl(Args args) {
    orderManager.motionControlSystem.setMaxRotationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_maxtrro::impl(Args args){
    orderManager.motionControlSystem.setMaxTranslationSpeed(OrderManager::parseFloat(args[0]));
    orderManager.motionControlSystem.setMaxRotationSpeed(OrderManager::parseFloat(args[1]));
}

void ORDER_av::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::FORWARD);
    orderManager.highLevel.printfln(DEBUG_HEADER, "av received");
}

void ORDER_rc::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::BACKWARD);
    orderManager.highLevel.printfln(DEBUG_HEADER, "rc received");
}

void ORDER_td::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::ANTITRIGO);
    orderManager.highLevel.printfln(DEBUG_HEADER, "td received");
}

void ORDER_tg::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::TRIGO);
    orderManager.highLevel.printfln(DEBUG_HEADER, "tg received");
}

void ORDER_trstop::impl(Args args)
{
    orderManager.motionControlSystem.stopTranslation();
}

void ORDER_rostop::impl(Args args)
{
    orderManager.motionControlSystem.stopRotation();
}

void ORDER_sstop::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::NONE);
    orderManager.highLevel.printfln(DEBUG_HEADER, "sstop received");
}

void ORDER_toggle::impl(Args args)
{
   /* FIXME orderManager.motionControlSystem.translation = !orderManager.motionControlSystem.translation;   //Bascule entre le réglage d'asserv en translation et en rotation
    if (orderManager.motionControlSystem.translation) {
        orderManager.highLevel.printfln(DEBUG_HEADER, "reglage de la translation");
    } else
        orderManager.highLevel.printfln(DEBUG_HEADER, "reglage de la rotation");
*/
}

void ORDER_displayAsserv::impl(Args args)
{
    /*
    float
            kp_t, ki_t, kd_t,      // Translation
            kp_r, ki_r, kd_r,      // Rotation
            kp_g, ki_g, kd_g,      // Vitesse gauche
            kp_d, ki_d, kd_d;      // Vitesse droite
            */
 /* FIXME   orderManager.motionControlSystem.getTranslationTunings(kp_t, ki_t, kd_t);
    orderManager.motionControlSystem.getRotationTunings(kp_r, ki_r, kd_r);
    orderManager.motionControlSystem.getLeftSpeedTunings(kp_g, ki_g, kd_g);
    orderManager.motionControlSystem.getRightSpeedTunings(kp_d, ki_d, kd_d);
    orderManager.highLevel.printfln(DEBUG_HEADER,"trans : kp= %g ; ki= %g ; kd= %g", kp_t, ki_t, kd_t);
    orderManager.highLevel.printfln(DEBUG_HEADER,"rot   : kp= %g ; ki= %g ; kd= %g", kp_r, ki_r, kd_r);
    orderManager.highLevel.printfln(DEBUG_HEADER,"gauche: kp= %g ; ki= %g ; kd= %g", kp_g, ki_g, kd_g);
    orderManager.highLevel.printfln(DEBUG_HEADER,"droite: kp= %g ; ki= %g ; kd= %g", kp_d, ki_d, kd_d);
*/
}

void ORDER_kpt::impl(Args args)
{
 /* FIXME   float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kp_trans ?");
    orderManager.motionControlSystem.getTranslationTunings(kp, ki, kd);
    kp = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setTranslationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kp_trans = %g", kp);
*/
}

void ORDER_kdt::impl(Args args)
{
   /* FIXME float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kd_trans ?");
    orderManager.motionControlSystem.getTranslationTunings(kp, ki, kd);
    kd = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setTranslationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kd_trans = %g", kd);
*/
}

void ORDER_kit::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"ki_trans ?");
    orderManager.motionControlSystem.getTranslationTunings(kp, ki, kd);
    ki = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setTranslationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"ki_trans = %g", ki);
*/
}

void ORDER_kpr::impl(Args args)
{
  /* FIXME  float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kp_rot ?");
    orderManager.motionControlSystem.getRotationTunings(kp, ki, kd);
    kp = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRotationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kp_rot = %g", kp);
*/
}

void ORDER_kir::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"ki_rot ?");
    orderManager.motionControlSystem.getRotationTunings(kp, ki, kd);
    ki = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRotationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"ki_rot = %g", ki);
*/}

void ORDER_kdr::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kd_rot ?");
    orderManager.motionControlSystem.getRotationTunings(kp, ki, kd);
    kd = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRotationTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kd_rot = %g", kd);
*/
}

void ORDER_kpg::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kp_gauche ?");
    orderManager.motionControlSystem.getLeftSpeedTunings(kp, ki, kd);
    kp = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setLeftSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kp_gauche = %g", kp);
*/}

void ORDER_kig::impl(Args args)
{
  /* FIXME  float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"ki_gauche ?");
    orderManager.motionControlSystem.getLeftSpeedTunings(kp, ki, kd);
    ki = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setLeftSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"ki_gauche = %g", ki);
*/}

void ORDER_kdg::impl(Args args)
{
 /* FIXME   float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kd_gauche ?");
    orderManager.motionControlSystem.getLeftSpeedTunings(kp, ki, kd);
    kd = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setLeftSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kd_gauche = %g", kd);
*/}

void ORDER_kpd::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kp_droite ?");
    orderManager.motionControlSystem.getRightSpeedTunings(kp, ki, kd);
    kp = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRightSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kp_droite = %g", kp);
*/
}

void ORDER_kid::impl(Args args)
{
  /* FIXME  float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"ki_droite ?");
    orderManager.motionControlSystem.getRightSpeedTunings(kp, ki, kd);
    ki = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRightSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"ki_droite = %g", ki);
*/
}

void ORDER_kdd::impl(Args args)
{
/* FIXME    float kp, ki, kd;
    orderManager.highLevel.printfln(STD_HEADER,"kd_droite ?");
    orderManager.motionControlSystem.getRightSpeedTunings(kp, ki, kd);
    kd = OrderManager::parseFloat(args[0]);
    orderManager.motionControlSystem.setRightSpeedTunings(kp, ki, kd);
    orderManager.highLevel.printfln(DEBUG_HEADER,"kd_droite = %g", kd);
*/
}


void ORDER_nh::impl(Args args)
{
    uint8_t id;
    int32_t x;
    uint32_t y, r;
    float angleHook, angleTolerance;
        id = (uint8_t) OrderManager::parseInt(args[0]);
        x = (int32_t) OrderManager::parseInt(args[1]);
        y = (uint32_t) OrderManager::parseInt(args[2]);
        r = (uint32_t) OrderManager::parseInt(args[3]);
        angleHook = OrderManager::parseFloat(args[4]);
        angleTolerance = OrderManager::parseFloat(args[5]);

        char hookOrder[RX_BUFFER_SIZE] = "";

        for (int i = 6; i < nbr_args; i++) {
            strcat(hookOrder, args[i]);
            strcat(hookOrder, " ");
        }
        hookOrder[RX_BUFFER_SIZE - 1] = '\0';

        orderManager.hookList.addHook(id, x, y, r, angleHook, angleTolerance, hookOrder);

        Serial.print("Ordre du hook: ");
        Serial.println(hookOrder);

        //TEST:
        Serial.println(orderManager.hookList.getHook(id).getOrder());
}

void ORDER_eh::impl(Args args)
{
    int hookId = OrderManager::parseInt(args[0]);
    if(orderManager.hookList.hookWithId(hookId))
    {
        orderManager.hookList.enableHook((uint8_t)hookId); //Singe proof ?
    }
    else
    {
        orderManager.highLevel.printfln(DEBUG_HEADER,"ERREUR::Activation d'un hook inexistant");
    }
}

void ORDER_dh::impl(Args args) {
    int hookId = OrderManager::parseInt(args[0]);

    if (orderManager.hookList.hookWithId(hookId)) {
        orderManager.hookList.disableHook((uint8_t) hookId); //Singe proof ?
    } else {
        orderManager.highLevel.printfln(DEBUG_HEADER, "ERREUR::Désactivation d'un hook inexistant");
    }
}

void ORDER_demo::impl(Args args) {

}

void ORDER_ptpdemo::impl(Args args)
{
    orderManager.execute("goto 500 -700");
    delay(5000);
    orderManager.execute("goto 1000 -400");
    delay(5000);
    orderManager.execute("goto 750 100");
    delay(5000);
    orderManager.execute("goto 0 0");
}


void ORDER_ptpdemoseq::impl(Args args)
{
    orderManager.execute("goto 500 -700 true");
    delay(5000);
    orderManager.execute("goto 1000 -400 true");
    delay(5000);
    orderManager.execute("goto 750 100 true");
    delay(5000);
    orderManager.execute("goto 0 0 true");
}


// TODO: pour les 2 qui suivent: électrovannes?



void ORDER_Valve::impl(Args args)
{
    uint8_t valve = OrderManager::parseInt(args[0]);
    BufferedData arg(2);
    bool state;

    if ( !strcmp(args[1],"on"))
        state = true;
    else if ( !strcmp(args[1],"off"))
        state = false;
    else
        orderManager.highLevel.printfln(STD_HEADER, "ERREUR::Il faut spécifier si on veut mettre la valve sur on ou off.");

    putData(valve, &arg);
    putData(state, &arg);

    switch(valve) {
        case 0:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_VALVE, &arg);
                break;
        case 1:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_VALVE, &arg);
                break;
        case 2:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_VALVE, &arg);
                break;
        case 3:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_VALVE, &arg);
                break;
        case 4:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_VALVE, &arg);
                break;
        case 5:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_VALVE, &arg);
                break;
        case 6:
                executeRPC(ID_MAIN, ID_ORDER_VALVE, &arg);
                break;
        default:
            orderManager.highLevel.printfln(STD_HEADER, "ERREUR::L'argument donné (%d) n'est pas un entier entre 0 et 6.", valve);
     }
}

void ORDER_DiodeOn::impl(Args args){

    executeRPC(ID_MAIN, 4, nullptr);
}
void ORDER_DiodeOff::impl(Args args){
    executeRPC(ID_MAIN, 5, nullptr);
}

void ORDER_FlagDown::impl(Args args) {
    #if defined(MAIN)
    executeRPC(ID_MAIN, ID_ORDER_FLAG_DOWN, nullptr);

    #elif defined(SLAVE)
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    Servo* motorFlag = manager.motFlag;
    motorFlag->write(0);
    #endif
}

void ORDER_FlagUp::impl(Args args) {
    #if defined(MAIN)
    executeRPC(ID_MAIN, ID_ORDER_FLAG_UP, nullptr);

    #elif defined(SLAVE)
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    Servo* motorFlag = manager.motFlag;
    motorFlag->write(90);
    #endif
}

void ORDER_BrasOut::impl(Args args) {

    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = manager.motor4;
    mot->changeLED(true);
    mot->setGoalAngle(90.0f);
}

void ORDER_BrasIn::impl(Args args) {

//    ActuatorsMgr& manager = ActuatorsMgr::Instance();
//#if defined(MAIN)
//    if ( !strcmp(args[1],"left"))
//        XL430* mot = manager.motor9;
//    else if (!strcmp(args[1],"right"))
//        XL430* mot = manager.motor10;
//#elif defined(SLAVE)
//
//#endif
//    mot->changeLED(true);
//    mot->setGoalAngle(0.0f);
}

void ORDER_Suck::impl(Args args) {

    uint8_t idPump = OrderManager::parseInt(args[0]);
    BufferedData arg(2);
    bool state;

    if ( !strcmp(args[1],"on"))
        state = true;
    else if ( !strcmp(args[1],"off"))
        state = false;
    else
        orderManager.highLevel.printfln(STD_HEADER, "ERREUR::Il faut spécifier si on veut mettre la pompe sur on ou off.");

    putData(idPump, &arg);
    putData(state, &arg);

    switch(idPump) {
        case 0:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_SUCK, &arg);
                break;
        case 1:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_SUCK, &arg);
                break;
        case 2:
                executeRPC(ID_SLAVE_AVANT, ID_ORDER_SUCK, &arg);
                break;
        case 3:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_SUCK, &arg);
                break;
        case 4:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_SUCK, &arg);
                break;
        case 5:
                executeRPC(ID_SLAVE_ARRIERE, ID_ORDER_SUCK, &arg);
                break;
        case 6:
                executeRPC(ID_MAIN, ID_ORDER_SUCK, &arg);
                break;
        default:
            orderManager.highLevel.printfln(STD_HEADER, "ERREUR::L'argument donné (%d) n'est pas un entier entre 0 et 6.", idPump);
     }
}

#if defined(MAIN)

void ORDER_LiftUp::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = manager.motor8;
    mot->toggleTorque(false);
    mot->setOperatingMode(4);
    mot->toggleTorque(true);

    mot->setGoalAngle(0.0f);


}
void ORDER_LiftDown::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = manager.motor8;
    mot->toggleTorque(false);
    mot->setOperatingMode(4);
    mot->toggleTorque(true);

    mot->setGoalAngle(400.0f);
}

void ORDER_Gate::impl(Args args)
{
    BufferedData arg(1);
    uint8_t angle = OrderManager::parseInt(args[0]);
    putData(angle, &arg);
    executeRPC(ID_MAIN, ID_ORDER_GATE, &arg);

}



#elif defined(SLAVE)

void ORDER_BrasStock::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = nullptr;
    switch (OrderManager::parseInt(args[0])){
        case 0:
            mot = manager.motor0;
            break;
        case 1:
            mot = manager.motor1;
            break;
        case 2:
            mot = manager.motor2;
            break;
        case 3:
            mot = manager.motor3;
            break;
        case 4:
            mot = manager.motor4;
            break;
        case 5:
            mot = manager.motor5;
            break;
    }
    if(mot) {
        mot->toggleTorque(true);
        mot->setGoalAngle(240.0f);
    }
}

void ORDER_BrasEcueil::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = nullptr;
    switch (OrderManager::parseInt(args[0])){
        case 0:
            mot = manager.motor0;
            break;
        case 1:
            mot = manager.motor1;
            break;
        case 2:
            mot = manager.motor2;
            break;
        case 3:
            mot = manager.motor3;
            break;
        case 4:
            mot = manager.motor4;
            break;
        case 5:
            mot = manager.motor5;
            break;
    }
    if(mot) {
        mot->toggleTorque(true);
        mot->setGoalAngle(165.0f);
    }
}

void ORDER_BrasDepot::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = nullptr;
    switch (OrderManager::parseInt(args[0])){
        case 0:
            mot = manager.motor0;
            break;
        case 1:
            mot = manager.motor1;
            break;
        case 2:
            mot = manager.motor2;
            break;
        case 3:
            mot = manager.motor3;
            break;
        case 4:
            mot = manager.motor4;
            break;
        case 5:
            mot = manager.motor5;
            break;
    }
    if(mot) {
        mot->toggleTorque(true);
        mot->setGoalAngle(140.0f);
    }
}

void ORDER_oust::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* mot = manager.motor4;
    mot->setGoalAngle(270);
}

void ORDER_grnd::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    Arm<XL430>* arm = manager.rightArm;
    arm->setPosition(positionIntermediaire);
    arm->setPosition(positionSolIntermediaire);
    arm->setPosition(positionSol);
}


#endif

void ORDER_XLm::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    uint8_t id = OrderManager::parseInt(args[0]);
    XL430* motor = (XL430*)manager.dynamixelManager->getMotor(id);
    motor->toggleTorque(true);
    motor->changeLED(true);
    motor->setGoalAngle(OrderManager::parseFloat(args[1]));
}

void ORDER_XLs::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* motor = (XL430*)manager.dynamixelManager->getMotor(OrderManager::parseInt(args[0]));
    motor->setGoalVelocity(OrderManager::parseFloat(args[1]));
}

void ORDER_posBras::impl(Args args) {
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    Arm<XL430>* arm = manager.rightArm;
    float angles[3];
    MOVE_ARM(args[0],
             arm->fetchAngles(angles));

    orderManager.highLevel.printfln(DEBUG_HEADER, "Angles are %f ; %f ; %f", angles[0], angles[1], angles[2]);
}




void ORDER_rangeSICK::impl(Args args) {
    uint8_t index = (uint8_t) OrderManager::parseInt(args[0]);
    uint16_t min = (uint16_t) OrderManager::parseInt(args[1]);
    uint16_t max = (uint16_t) OrderManager::parseInt(args[2]);
    if(index < NBR_OF_DISTANCE_SENSOR) {
        SensorMgr::Instance().getDistanceSensor(index).setRange(min, max);
    } else {
        orderManager.highLevel.printf(DEBUG_HEADER, "Aucun SICK à l'indice %i!\n", index);
    }
    orderManager.highLevel.printf(DEBUG_HEADER, "Le SICK %i est maintenant dans l'intervalle [%i; %i]\n", index, min, max);
}

void ORDER_testSICK::impl(Args args) {
    if(args.size() > 0) {
        uint8_t index = (uint8_t) OrderManager::parseInt(args[0]);
        if(index < NBR_OF_DISTANCE_SENSOR) {
            orderManager.highLevel.printf(SICK_HEADER, "%i\n", SensorMgr::Instance().getDistanceSensor(index).readDistance());
        } else {
            orderManager.highLevel.printf(DEBUG_HEADER, "Aucun SICK à l'indice %i!\n", index);
        }
    }
}

// TODO : Tester si avec un for et des printfln("", "%d "); ça marcherait
void ORDER_lectureSICK::impl(Args args) {
    SensorMgr mgr = SensorMgr::Instance();
    if(NBR_OF_DISTANCE_SENSOR == 3) {
        orderManager.highLevel.printfln(SICK_HEADER, "%d %d %d",
                                        mgr.getDistanceSensor(0).readDistance(),
                                        mgr.getDistanceSensor(1).readDistance(),
                                        mgr.getDistanceSensor(2).readDistance());
    } else {
        orderManager.highLevel.printfln(SICK_HEADER, "%d %d %d %d %d %d",
                                        mgr.getDistanceSensor(0).readDistance(),
                                        mgr.getDistanceSensor(1).readDistance(),
                                        mgr.getDistanceSensor(2).readDistance(),
                                        mgr.getDistanceSensor(3).readDistance(),
                                        mgr.getDistanceSensor(4).readDistance(),
                                        mgr.getDistanceSensor(5).readDistance());
    }
}

void ORDER_disableTorque::impl(Args args) {
    ActuatorsMgr &manager = ActuatorsMgr::Instance();
    MOVE_ARM(args[0],
             arm->setTorque(false);
    )
}

void ORDER_enableTorque::impl(Args args) {
    ActuatorsMgr &manager = ActuatorsMgr::Instance();
    MOVE_ARM(args[0],
             arm->setTorque(true);
    )
}

void ORDER_debugAsserv::impl(Args args) {
    BufferedData answer(sizeof(int16_t) * 2);
    dataRequest(MCS_SLAVE_ID, DEBUG_RPC_ID, answer, nullptr);
    int16_t leftPWM;
    int16_t rightPWM;
    getData<int16_t>(leftPWM, &answer);
    getData<int16_t>(rightPWM, &answer);
    Serial.print(">> ");
    Serial.print(leftPWM);
    Serial.println();
    Serial.print(">> ");
    Serial.print(rightPWM);
    Serial.println();
}

void ORDER_torqueBras::impl(Args args)
{
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    // TODO Arm* arm = !strcmp(args[0], "right") ? manager.rightArm : manager.leftArm;
    // TODO
    
    /*int couple[3] = {0, 0, 0};
    if (!strcmp(args[1], "sol"))
    {
        for ( int i = 0 ; i < 3 ; i++ )
        { // Pour chaque XL
            XL430 motor = arm->getXLlist()[i];
            if (motor.getCurrentTorque(couple[i]))
            { // renvoit true si la mesure a été effectuée

                // Pour chaque couleur de la plus lourde à la plus légère
                for (int color = (int)PaletColor::GOLD; color < (int)PaletColor::NONE ; color++ )
                {
                    if (couple[i] > coupleSolseuil[i][color])
                    { //test de chaque palet
                        orderManager.highLevel.printfln(ATOM_COLOR_HEADER, "%s", PaletColorToString((PaletColor)color).c_str());
                        break;
                    }
                }
                orderManager.highLevel.printfln(DEBUG_HEADER, "palet non pris");
            }
            else
            {
                orderManager.highLevel.printfln(DEBUG_HEADER, "torque failed");
            }
        }
    }
    else
    {
        for (int i = 0; i < 3; i++)
        { // Pour chaque XL
            XL430 motor = arm->getXLlist()[i];
            if (motor.getCurrentTorque(couple[i]))
            { // renvoit true si la mesure a été effectuée

                // Pour chaque couleur de la plus lourde à la plus légère
                for (int color = (int)PaletColor::GOLD; color < (int)PaletColor::NONE ; color++ )
                {
                    if (couple[i] > coupleDistributeurseuil[i][color])
                    { //test de chaque palet
                        orderManager.highLevel.printfln(ATOM_COLOR_HEADER, "%s",  PaletColorToString((PaletColor)color).c_str());
                        break;
                    }
                }
                orderManager.highLevel.printfln(DEBUG_HEADER, "palet non pris");
            }
            else
            {
                orderManager.highLevel.printfln(DEBUG_HEADER, "torque failed");
            }
        }
    }*/
}

void ORDER_torqueXL :: impl(Args args){
    /* TODO
    ActuatorsMgr& manager = ActuatorsMgr::Instance();
    XL430* motor = (XL430*)manager.dynamixelManager->getMotor(OrderManager::parseInt(args[0]));
    int couple;
    if(motor->getCurrentTorque(couple)) {
        orderManager.highLevel.printfln(SENSOR_HEADER,"%i",couple);
    }
    else{
        orderManager.highLevel.printfln(DEBUG_HEADER,"%s","couple failed");
    }*/
}

void ORDER_waitJumper::impl(Args args) {
    Serial.println("Waiting for jumper...");

#if defined(MAIN)
    //digitalWrite(LED1, HIGH);
#elif defined(SLAVE)
    //digitalWrite(LED1_1, LOW);
#endif

    // attente de front
    while(digitalRead(PIN_JMPR) == HIGH) {
        InterruptStackPrint::Instance().print();
    }
    while(digitalRead(PIN_JMPR) == LOW) {
        InterruptStackPrint::Instance().print();
    }
    ComMgr::Instance().printfln(EVENT_HEADER, "gogogofast");
#if defined(MAIN)
    //digitalWrite(LED1, LOW);
#elif defined(SLAVE)
//    digitalWrite(LED1_1, HIGH);
//    digitalWrite(LED1_2, LOW);
#endif
}

void ORDER_endMatch::impl(Args args) {
#if defined(MAIN)
    digitalWrite(LEFT_PUMP_PIN, LOW);
    digitalWrite(LEFT_VALVE_PIN, HIGH);
#endif
    digitalWrite(RIGHT_PUMP_PIN, LOW);
    digitalWrite(RIGHT_VALVE_PIN, HIGH);
    orderManager.execute("stop");
    orderManager.execute("sstop");

#if defined(MAIN)

    while(true) {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, LOW);
        delay(100);
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, HIGH);
        delay(100);
    }

#elif defined(SLAVE)

    digitalWrite(LED1_3, HIGH);
    digitalWrite(LED2_1, HIGH);
    digitalWrite(LED2_2, HIGH);
    digitalWrite(LED3_1, HIGH);
    digitalWrite(LED3_2, HIGH);
    while(true) {
        digitalWrite(LED1_1, LOW);
        digitalWrite(LED1_2, LOW);
        digitalWrite(LED2_3, LOW);
        digitalWrite(LED3_3, LOW);
        delay(100);
        digitalWrite(LED1_1, HIGH);
        digitalWrite(LED1_2, HIGH);
        digitalWrite(LED2_3, HIGH);
        digitalWrite(LED3_3, HIGH);
        delay(100);
    }

#endif
}