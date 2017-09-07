/**
SS		10
INT		/
RST		14
PWDN	15
MISO	12
MOSI	11
*/


#ifndef _ETHERNETMGR_h
#define _ETHERNETMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include "Singleton.hpp"
#include "stdarg.h"
#include <WString.h>
#include "defines.h"
#include <Ethernet.h>
#include "pin_mapping.h"

#define RX_TIMEOUT 1000 //TODO g�rer les timeouts(surtout pour le moniteur s�rie..)

#define HEADER_LENGTH   2

#define DEBUG_HEADER    {0x02, 0x20}
#define EVENT_HEADER    {0x13, 0x37}
#define US_HEADER       {0x01, 0x10}

#define PORT 23500

class EthernetMgr : public Singleton<EthernetMgr>
{
private:
	void resetCard();
	bool read_char(byte &);
	/* Attributs Ethernet */
	EthernetServer server{ PORT };
	EthernetClient client;

public:
	EthernetMgr();
	enum {
		READ_TIMEOUT = 0, READ_SUCCESS = 1
	};

	bool connected;

	/* RECEPTION */
	bool read(String&);
	bool read(int16_t&);
	bool read(volatile int8_t &);
	bool read(float&);

	/* ENVOI */
	void sendUS(uint16_t[]);
	void printfln(const char*, ...) __attribute__((format(printf, 2, 3)));
	void log(const char*, ...) __attribute__((format(printf, 2, 3)));
};
#endif