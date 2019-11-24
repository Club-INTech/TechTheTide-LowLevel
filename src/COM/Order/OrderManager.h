/**
*Contient la liste des correspondances pour les entrées série du programme
*
* @author caillou, sylvain, rémi, melanie, Ug
*
**/

#ifndef _ORDERMGR_h
#define _ORDERMGR_h

#include "Utils/Metro.h"
#include <map>
#include <set>
#include <string>
#include <cstdlib>
#include <Arduino.h>
#include <WString.h>
#include "Utils/Singleton.hpp"
#include "MotionControlSystem/MCS.h"
#include "Sensors/SensorMgr.h"
#include "COM/ComMgr.h"
#include "Actuators/ActuatorsMgr.h"
#include "Config/Defines.h"
#include "Utils/Utils.h"
#include "COM/Hooks/Hook.h"
#include "OrderData.h"
#include "Orders.h"
#include "COM/Hooks/HookList.h"

class OrderManager : public Singleton<OrderManager>
{
public:
	HookList hookList;
	OrderData orderData;
	SensorMgr &sensorMgr;
	MCS& motionControlSystem;
	ActuatorsMgr &actuatorsMgr;
	ComMgr& highLevel;

    //Variables booleennes pour envoi de données au HL
    bool isSendingUS;
	bool HLWaiting;

    OrderManager();

    void init();

    /**
     * Enregistres un ordre avec l'id correspondant
     * @param id l'identifiant de l'ordre
     * @param order l'ordre à enregistrer
     */
    void registerOrder(String id, AbstractOrder* order);

    //COM&exec
    inline void refreshUS(){
	}
	inline void sendUS(){
	}
    void communicate();
    void execute(const char*);	//public pour pouvoir executer des scripts de hook


    //Utilitaire
    int8_t split(char* , OrderData& , const char* separator = " ");
    int parseInt(const char*);
    float parseFloat(const char*);
	bool isHLWaiting();
	void checkJumper();

    //Hooks
    void checkHooks();
    void executeHooks();
    bool hooksEnabled;

//    std::map<std::string,int> lookupTable;


private:

    std::map<String, AbstractOrder*> orders;

    char readMessage[RX_BUFFER_SIZE];
    char charIDLastMessage;
    uint64_t messageCount = 0;
};

#endif //_ORDERMGR_h
