//
// Created by asphox on 29/04/18.
//

#ifndef TECHTHETOWN_LOWLEVEL_ORDERS_H
#define TECHTHETOWN_LOWLEVEL_ORDERS_H

#include "AbstractOrder.h"
#include "OrderManager.h"
#include "Utils/Utils.h"
#include "Config/PinMapping.h"
#include "MotionControlSystem/RobotStatus.h"
#include "Actuators/ActuatorValues.h"
#include "Actuators/ActuatorsMgr.h"

ORDER(ping,0);
ORDER(j,0);
ORDER(f,0);
ORDER(xyo,0);
ORDER(d,1);
ORDER(t,1);
ORDER(goto,2);
ORDER(followTrajectory,1);
ORDER(stop,0);

/*			 __________________
* 		   *|                  |*
*		   *|   POS & VITESSE  |*
*		   *|__________________|*
*/

ORDER(cx,1);
ORDER(cy,1);
ORDER(co,1);
ORDER(cxyo,3);
ORDER(ctv,1);
ORDER(crv,1);
ORDER(ctrv,2);
ORDER(efm,0);
ORDER(dfm,0);

/*			 ___________________
* 		   *|                   |*
*		   *|  ASSERVISSEMENTS  |*
*		   *|___________________|*
*/

ORDER(ct0,0);
ORDER(ct1,0);
ORDER(cr0,0);
ORDER(cr1,0);
ORDER(cv0,0);
ORDER(cv1,0);

/*			 ___________________________
* 		   *|                           |*
*		   *|					        |*
*		   *|			DEBUG			|*
*		   *|						    |*
*		   *|___________________________|*
*/

ORDER(cod,0);
ORDER(pfdebug,0);
ORDER(rawpwm,1);
ORDER(getpwm,0);
ORDER(errors,0);
ORDER(rawspeed,0);
ORDER(rawposdata,0);
ORDER(reseteth,0);

ORDER(disableTorque, 1);
ORDER(enableTorque, 1);

/*			 ___________________________
* 		   *|                           |*
*		   *|         MONTLHERY         |*
*		   *|   DEPLACEMENT ET ROTATION |*
*		   *|    SANS ASSERVISSEMENT    |*
*		   *|___________________________|*
*/

ORDER(montlhery,0);
ORDER(av,0);
ORDER(rc,0);
ORDER(td,0);
ORDER(tg,0);
ORDER(sstop,0);
ORDER(maxtr,1);
ORDER(maxro,1);
ORDER(maxtrro,2);
ORDER(trstop,0);
ORDER(rostop,0);

/*			 _________________________________
* 		   *|                                 |*
*		   *|CONSTANTES D'ASSERV (pour le PID)|*
*    	   *|_________________________________|*
*/

ORDER(toggle,0);
ORDER(displayAsserv,0);
ORDER(kpt,1);
ORDER(kdt,1);
ORDER(kit,1);
ORDER(kpr,1);
ORDER(kir,1);
ORDER(kdr,1);
ORDER(kpg,1);
ORDER(kig,1);
ORDER(kdg,1);
ORDER(kpd,1);
ORDER(kid,1);
ORDER(kdd,1);

/*			 _________________________________
* 		   *|                                 |*
*		   *|			   HOOKS	          |*
*    	   *|_________________________________|*
*/

ORDER(nh,7);
ORDER(eh,1);
ORDER(dh,1);

/*			 _________________________________
* 		   *|                                 |*
 *		   *|	       ACKNOWLEDGEMENT        |*
 *    	   *|_________________________________|*
*/

ORDER(demo,0);
ORDER(ptpdemo,0);
ORDER(ptpdemoseq,0);

/*			 _________________________________
* 		   *|                                 |*
 *		   *|	      NOUVEAUX ORDRES         |*
 *    	   *|_________________________________|*
*/


/* Bras */
ORDER(XLm,2);
ORDER(XLs,2);
ORDER(posBras,1);

// This is horrible and should probably burn
#if defined(MAIN)

ORDER(GateOpen,1);


ORDER(torqueBras,1);
ORDER(torqueXL,1);


#elif defined(SLAVE)

ORDER(grnd,1);

ORDER(torqueBras,1);
ORDER(torqueXL,1);

/* Pousse palet en dehors des ascenceurs */
ORDER(oust,0);

#endif



/* SICK */
ORDER(testSICK, 1);
ORDER(rangeSICK, 3);
ORDER(lectureSICK, 0);

ORDER(waitJumper, 0);
ORDER(endMatch, 0);

// Permet de bouger un bras sans se soucier du type
#if defined(MAIN)
#define MOVE_ARM(side, actions)             \
if(!strcmp(args[0], "right")) {             \
    Arm<XL430>* arm = manager.rightArm;      \
    actions;                                \
} else {                                    \
    Arm<XL430>* arm = manager.leftArm;      \
    actions;                                \
}
#elif defined(SLAVE)
#define MOVE_ARM(side, actions)             \
if(!strcmp(args[0], "right")) {             \
    Arm<XL430>* arm = manager.rightArm;      \
    actions;                                \
}
#endif


#endif //TECHTHETOWN_LOWLEVEL_ORDERS_H
