#include "DynamixelMotor.h"

DynamixelDevice::DynamixelDevice(DynamixelInterface &aInterface, const DynamixelID aID):
	mInterface(aInterface), mStatusReturnLevel(255), mID(aID)
{
	mStatus = DYN_STATUS_OK;
	if (mID == BROADCAST_ID)
	{
		mStatusReturnLevel = 0;
	}
}

uint8_t DynamixelDevice::statusReturnLevel()
{
	if (mStatusReturnLevel == 255)
	{
		init();
	}
	return mStatusReturnLevel;
}

void DynamixelDevice::statusReturnLevel(uint8_t aSRL)
{
	write(DYN_ADDRESS_SRL, aSRL);
	if (status() == DYN_STATUS_OK)
	{
		mStatusReturnLevel = aSRL;
	}
}

uint16_t DynamixelDevice::model()
{
	uint16_t result;
	read(DYN_ADDRESS_ID, result);
	return result;
}

uint8_t DynamixelDevice::firmware()
{
	uint8_t result;
	read(DYN_ADDRESS_FIRMWARE, result);
	return result;
}

void DynamixelDevice::communicationSpeed(uint32_t aSpeed)
{
	uint8_t value = 2000000 / aSpeed - 1;
	if (value != 0) // forbid 2MBd rate, because it is out of spec, and can be difficult to undo
	{
		write(DYN_ADDRESS_BAUDRATE, value);
	}
}

DynamixelStatus DynamixelDevice::init()
{
	mStatusReturnLevel = 0;
	DynamixelStatus status = ping();
	if (status != DYN_STATUS_OK)
	{
		return status;
	}
	status = read(DYN_ADDRESS_SRL, mStatusReturnLevel);
	if (status & DYN_STATUS_TIMEOUT)
	{
		mStatusReturnLevel = 0;
	}
	return DYN_STATUS_OK;
}



DynamixelMotor::DynamixelMotor(DynamixelInterface &aInterface, const DynamixelID aId):
	DynamixelDevice(aInterface, aId)
{}


void DynamixelMotor::wheelMode()
{
	jointMode(0,0);
}

void DynamixelMotor::jointMode(uint16_t aCWLimit, uint16_t aCCWLimit)
{
	uint32_t data = (aCWLimit | (uint32_t(aCCWLimit) << 16));
	write(DYN_ADDRESS_CW_LIMIT, data);
}

void DynamixelMotor::enableTorque(bool aTorque)
{
	write(DYN_ADDRESS_ENABLE_TORQUE, uint8_t(aTorque?1:0));
}

DynamixelStatus DynamixelMotor::alarmShutdown(uint8_t aMode)
{
	aMode &= B01111111;
	return write(DYN_ADDRESS_ALARM_SHUTDOWN, aMode);
}

DynamixelStatus DynamixelMotor::speed(const uint16_t aSpeed)
{
	return write(DYN_ADDRESS_GOAL_SPEED, aSpeed);
}

DynamixelStatus DynamixelMotor::torqueLimit(const uint16_t aTorque)
{
	return write(DYN_ADDRESS_TORQUE_LIMIT, aTorque);
}

DynamixelStatus DynamixelMotor::goalPosition(const uint16_t aPosition)
{
	return write(DYN_ADDRESS_GOAL_POSITION, aPosition);
}

DynamixelStatus DynamixelMotor::goalPositionWait(const uint16_t aPosition)
{
 	DynamixelStatus status = write(DYN_ADDRESS_GOAL_POSITION, aPosition);
	if( status == DynStatus::DYN_STATUS_OK )
	{
		while( currentPosition() != aPosition )
		{
		}
	}
	return status;
}

DynamixelStatus DynamixelMotor::goalPositionDegree(const uint16_t posDeg)
{
	return goalPosition(posDeg * 3.41);
}

DynamixelStatus DynamixelMotor::goalPositionDegreeWait(const uint16_t posdeg)
{
	DynamixelStatus status = write(DYN_ADDRESS_GOAL_POSITION, posdeg);
	if( status == DynStatus::DYN_STATUS_OK )
	{
		while( currentPositionDegree() != posdeg )
		{
		}
	}
	return status;
}

void DynamixelMotor::setId(const uint8_t newId)
{
	write(DYN_ADDRESS_ID, newId);
	mID = newId;
}

void DynamixelMotor::led(const uint8_t aState)
{
	write(DYN_ADDRESS_LED, aState);
}

uint16_t DynamixelMotor::currentPosition()
{
	uint16_t currentPosition;
	if (read(DYN_ADDRESS_CURRENT_POSITION, currentPosition) == 0)
	{
		return currentPosition;
	}
	else
	{
		return UINT16_MAX;
	}
}

uint16_t DynamixelMotor::currentPositionDegree()
{
	return (uint16_t)((float)currentPosition() / 3.41);
}
