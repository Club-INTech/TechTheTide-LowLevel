//
// Created by trotfunky on 19/02/18.
//

#ifndef LL_ACTUTORVALUES_H
#define LL_ACTUTORVALUES_H

/*
 * AX12-IDs
 */

constexpr int frontArmGroup_ID = 0;
constexpr int frontLeftAX12_ID = 1;
constexpr int frontRightAX12_ID = 2;

constexpr int backArmGroup_ID = 0;
constexpr int backLeftAX12_ID = 4;
constexpr int backRightAX12_ID = 5;

constexpr int frontDoor_ID = 3;
constexpr int backDoor_ID = 6;


constexpr int armBaseSpeed = 140;

/*
 * Bras avant
 */

constexpr int frontLowCubes = 95;
constexpr int frontLowBee = 140;
constexpr int frontFolded = 185;

/*
 * Bras arrière
 */

constexpr int backLowCubes = 202;
constexpr int backFolded = 120;

/*
 * Portes
 */

constexpr int doorBaseSpeed = 200;

constexpr int frontDoorOpen = 240;
constexpr int frontDoorClosed = 155;

constexpr int backDoorOpen = 150;
constexpr int backDoorClosed = 245;


#endif //LL_ACTUTORVALUES_H