//
// Created by asphox on 24/04/18.
//

#ifndef TECHTHETOWN_LOWLEVEL_ABSTRACTCOMINTERFACE_H
#define TECHTHETOWN_LOWLEVEL_ABSTRACTCOMINTERFACE_H

#include <Arduino.h>
#include <vector>
#include "Utils/Average.hpp"
#include "Utils/stdarg.h"

class AbstractComInterface
{
public:
    AbstractComInterface() = default;
    virtual ~AbstractComInterface() = default;

    virtual bool read(char*) = 0;
    virtual bool read(int32_t&) = 0;
    virtual bool read(int16_t&) = 0;
    virtual bool read(volatile int8_t &) = 0;
    virtual bool read(float&) = 0;

    virtual void printfln(const char*) = 0;
    virtual void printf(const char*) = 0;


protected:
    String data;
};


#endif //TECHTHETOWN_LOWLEVEL_ABSTRACTCOMINTERFACE_H
