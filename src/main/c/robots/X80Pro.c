#include "boards/PMS5005.h"
#include "sockets/UDPSocket.h"
#include "X80Pro.h"

static PMS5005_t PMS5005;

static void motorSensorRequest(X80Pro *self, short packetNumber)
{
	self->socket.send(&self->socket, PMS5005.motorSensorRequest(Buffer buf, packetNumber));
}

static void standardSensorRequest(X80Pro *self, short packetNumber)
{
	self->socket.send(&self->socket, PMS5005.standardSensorRequest(Buffer buf, packetNumber));
}

static void customSensorRequest(X80Pro *self, short packetNumber)
{
	self->socket.send(&self->socket, PMS5005.customSensorRequest(Buffer buf, packetNumber));
}

static void allSensorRequest(X80Pro *self, short packetNumber)
{
	self->socket.send(&self->socket, PMS5005.allSensorRequest(Buffer buf, packetNumber));
}

static void enableMotorSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.enableMotorSensorSending());
}

static void enableStandardSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.enableStandardSensorSending());
}

static void enableCustomSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.enableCustomSensorSending());
}

static void enableAllSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.enableAllSensorSending());
}

static void disableMotorSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.disableMotorSensorSending());
}

static void disableStandardSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.disableStandardSensorSending());
}

static void disableCustomSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.disableCustomSensorSending());
}

static void disableAllSensorSending(X80Pro *self)
{
	self->socket.send(&self->socket, PMS5005.disableAllSensorSending());
}

static setIRCtrlOutput(X80Pro *self, short loWord, short hiWord)
{
	self->socket.send(&self->socket, PMS5005.setIRCtrlOutput(Buffer buf, loWord, hiWord));
}

static short getSensorBatteryAD(X80Pro *self, short channel)
{
	return self->standardSensorData[2*channel + PMS5005.BATTERY_SENSOR_OFFSET + 1] << 8 | self->standardSensorData[2*channel + PMS5005.BATTERY_SENSOR_OFFSET];
}

static short getSensorRefVoltage(X80Pro *self)
{
	return self->standardSensorData[PMS5005.REFERENCE_VOLTAGE_OFFSET + 1] << 8 | self->standardSensorData[REFERENCE_VOLTAGE_OFFSET];
}

static short getSensorPotVoltage(X80Pro *self)
{
    return self->standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET + 1] << 8 | self->standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET];
}

static short getMotorCurrent(X80Pro *self, short channel)
{
    return (self->motorSensorData[2*channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET + 1] << 8 | self->motorSensorData[2*channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET]) / 728.0;
}

static short getEncoderDirection(X80Pro *self, short channel)
{
    int offset = channel + ENCODER_DIRECTION_OFFSET;
    short result = -1;
    
	return = self->motorSensorData[offset] & (0x01 + (0x02*channel));
/*
    switch (channel)
    {
	case 0:
		result = self->motorSensorData[offset] & 0x01;
		break;
	case 1:
		result = self->motorSensorData[offset] & 0x03;
		break;
    }
    
    return result;
*/
}

static short getEncoderPulse(X80Pro *self, short channel)
{
	int offset = 4*channel + PMS5005.MOTOR_SPEED_OFFSET;
	return self->motorSensorData[offset + 1] << 8 | self->motorSensorData[offset];
}

static short getEncoderSpeed(X80Pro *self, short channel)
{
	int offset = 4*channel + PMS5005.MOTOR_SPEED_OFFSET;
	return self->motorSensorData[offset + 1] << 8 | self->motorSensorData[offset];
}

static short getCustomAD(X80Pro *self, short channel)
{
	int offset = 2*channel + CUSTOM_AD_OFFSET;
	return self->customSensorData[offset + 1] << 8 | self->customSensorData[offset];
}

static short getCustomDIn(X80Pro *self, unsigned char channel)
{
	return 0;
}

static void setCustomDOut(X80Pro *self, unsigned char ival)
{
	/* stub */
}

static void setMotorPolarity(X80Pro *self, unsigned char channel, unsigned char polarity)
{
	self->socket.send(&self->socket, PMS5005.setMotorPolarity(Buffer buf, channel, polarity));
}

static void enableDCMotor(X80Pro *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.enableDCMotor(Buffer buf, channel));
}

static void disableDCMotor(X80Pro *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.disableDCMotor(Buffer buf, channel));
}

static void resumeDCMotor(X80PRo *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.resumeDCMotor(Buffer buf, channel));
}

static void suspendDCMotor(X80Pro *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.suspendDCMotor(Buffer buf, channel));
}

static void setDCMotorPositionPID(X80Pro *self, unsigned char channel, short kp, short kd, short ki_x100)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorPositionPID(Buffer buf, channel));
}

static void setDCMotorVelocityPID(X80Pro *self, unsigned char channel, short kp, short kd, short ki_x100)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorVelocityPID(Buffer buf, channel, kp, kd, ki_x100));
}

static void setDCMotorSensorFilter(X80Pro *self, unsigned char channel, short filterMethod)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorSensorFilter(Buffer buf, channel, filterMethod));
}

static void setDCMotorSensorUsage(X80Pro *self, unsigned char channel, unsigned char sensorType)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorSensorUsage(Buffer buf, channel, sensorType));
}

static void setDCMotorCtrlMode(X80Pro *self, unsigned char channel, unsigned char controlMode)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorCtrlMode(Buffer buf, channel, controlMode));
}

static void setDCMotorPositionWithTime(X80Pro *self, unsigned char channel, short pos, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorPositionWithTime(Buffer buf, unsigned char channel, short pos, short timePeriod));
}

static void setDCMotorPosition(X80Pro *self, unsigned char channel, short packetValue)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorPosition(Buffer buf, channel, packetValue));
}

static void setDCMotorPositionWithTime(X80Pro *self, unsigned char channel, short pulseWidth, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorPulseWithTime(Buffer buf, channel, pulseWith, timePeriod));
}

static void setDCMotorPulse(X80Pro *self, unsigned char channel, short pulseWidth)
{
	self->socket.send(&self->socket, PMS5005.setDCMotorPulse(Buffer buf, channel, pulseWidth));
}

static void setAllDCMotorPositionsWithTime(X80Pro *self, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPositionsWithTime(Buffer buf, pos0, pos1, pos2, pos3, pos4, pos5, timePeriod));
}

static void setAllDCMotorPositions(X80Pro *self, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulse(Buffer buf, pos0, pos1, pos2, pos3, pos4, pos5));
}

static void setBothDCMotorPositionsWithTime(X80Pro *self, short posLeft, short posRight, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulse(Buffer buf, posLeft, posRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void setBothDCMotorPositionsWithTime(X80Pro *self, short posleft, short PosRight)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPositionsWithTime(Buffer buf, posLeft, posRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void setAllDCMotorVelocitiesWithTime(X80Pro *self, short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorVelocitiesWithTime(Buffer buf, v0, v1, v2, v3, v4, v5, timePeriod));
}

static void setAllDCMotorVelocities(X80Pro *self, short v0, short v1, short v2, short v3, short v4, short v5)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorVelocities(Buffer buf, v0, v1, v2, v3, v4, v5));
}

static void setBothDCMotorVelocitiesWithTime(X80Pro *self, short vLeft, short vRight, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorVelocitiesWithTime(Buffer buf, vLeft, vRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, timePeriod));
}

static void setBothDCMotorVelocities(X80Pro *self, short vLeft, short vRight)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorVelocitiesWithTime(Buffer buf, vLeft, vRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void setAllDCMotorPulsesWithTime(X80Pro *self, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulsesWithTime(Buffer buf, p0, p1, p2, p3, p4, p5, timePeriod));
}

static void setAllDCMotorPulses(X80Pro *self, short p0, short p1, short p2, short p3, short p4, short p5)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulses(Buffer buf, p0, p1, p2, p3, p4, p5));
}

static void setBothDCMotorPulsesWithTime(X80Pro *self, short pLeft, short pRight, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulsesWithTime(Buffer buf, pLeft, pRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void setBothDCMotorPulses(X80Pro *self, short pLeft, short pRight)
{
	self->socket.send(&self->socket, PMS5005.setAllDCMotorPulses(Buffer buf, pLeft, pRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void enableServo(X80Pro *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.enableServo(Buffer buf, channel));
}

static void disableServo(X80Pro *self, unsigned char channel)
{
	self->socket.send(&self->socket, PMS5005.disableServo(Buffer buf, channel));
}

static void setServoPulseWithTime(X80Pro *self, unsigned char channel, short pulseWidth, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setServoPulseWithTime(Buffer buf, channel, pulseWidth, timePeriod));
}

static void setServoPulse(X80Pro *self, unsigned char channel, short pulseWidth)
{
	self->socket.send(&self->socket, PMS5005.setServoPulse(Buffer buf, channel, pulseWidth));
}

static void setAllServoPulsesWithTime(X80Pro *self, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllServoPulsesWithTime(Buffer buf, p0, p1, p2, p3, p4, p5, timePeriod));
}

static void setAllServoPulses(X80Pro *self, short p0, short p1, short p2, short p3, short p4, short p5)
{
	self->socket.send(&self->socket, PMS5005.setAllServoPulsesWithTime(Buffer buf, p0, p1, p2, p3, p4, p5));
}

static void setBothServoPulsesWithTime(X80Pro *self, short pLeft, short pRight, short timePeriod)
{
	self->socket.send(&self->socket, PMS5005.setAllServoPulsesWithTime(Buffer buf, pLeft, pRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, timePeriod));
}

static void setBothServoPulses(X80Pro *self, short pLeft, short pRight)
{
	self->socket.send(&self->socket, PMS5005.setAllServoPulses(Buffer buf, pLeft, pRight, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL, PMS5005.NO_CTRL));
}

static void setLCDDisplayPMS(X80Pro *self, char *bmpFileName)
{
	self->socket.send(&self->socket, PMS5005.setLCDDisplayPMS(Buffer buf, bmpFileName));
}

/* ======================================================================== */
/* === Sensor Methods ===================================================== */
/* ======================================================================== */

/**
 * Sets the refresh rate for batch updating by motor-related sensor packets.
 *
 * @param timePeriod The update period in milliseconds for batch sensing
 * packets to the PC central controller.
 *
 * Please note: The default timePeriod is 50ms.  The maximum refresh rate
 * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
 *
 * @see motorSensorRequest
 */
static void setMotorSensorPeriod(short timePeriod)
{
	/* stub */
}

/**
 * Sets the refresh rate for batch updating by custom sensor packets.
 *
 * @param timePeriod The update period in milliseconds for batch sensing
 * packets to the PC central controller.
 *
 * Please note: The default timePeriod is 50ms.  The maximum refresh rate
 * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
 *
 * @see customSensorRequest
 */
static void setStandardSensorPeriod(short timePeriod)
{
	/* stub */
}

/**
 * Sets the refresh rate for batch updating by custom sensor packets.
 *
 * @param timePeriod The update period in milliseconds for batch sensing
 * packets to the PC central controller.
 *
 * Please note: The default timePeriod is 50ms.  The maximum refresh rate
 * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
 *
 * @see customSensorRequest
 */
static void setCustomSensorPeriod(short timePeriod)
{
	/* stub */
}

/**
 * Sets the refresh rate for batch updating of all sensor packets.
 *
 * @param timePeriod The update period in milliseconds for batch sensing
 * packets to the PC central controller.
 *
 * Please note: The default timePeriod is 50ms.  The maximum refresh rate
 * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
 *
 * @see allSensorRequest
 */
static void setAllSensorPeriod(short timePeriod)
{
	/* stub */
}

static short getSensorSonar(X80Pro *self, const int *standardSensorData, short channel)
{
	return PMS5005.test();
}

static short getSensorIRRange(X80Pro *self, const int *standardSensorData, const int *customSensorData, short channel)
{
	return 0;
}

static short getSensorHumanAlarm(X80Pro *self, const int *standardSensorData, short channel)
{
	return 0;
}

static short getSensorHumanMotion(X80Pro *self, const int *standardSensorData, short channel)
{
	return 0;
}

static short getSensorTiltingX(X80Pro *self, const int *standardSensorData)
{
	return 0;
}

static short getSensorTiltingY(X80Pro *self, const int *standardSensorData)
{
	return 0;
}

static short getSensorOverheat(X80Pro *self, const int *standardSensorData, short channel)
{
	return 0;
}

static short getSensorTemperature(X80Pro *self, const int *standardSensorData)
{
	return 0;
}

static short getSensorIRCode(X80Pro *self, const int *standardSensorData, short index)
{
	return 0;
}

static short getSensorBatteryAD(X80Pro *self, const int *standardSensorData, short channel)
{
	return 0;
}

static short getSensorRefVoltage(X80Pro *self, const int *standardSensorData)
{
	return 0;
}

static short getSensorPotVoltage(X80Pro *self, const int *standardSensorData)
{
	return 0;
}

static short getSensorPot(X80Pro *self, const int *motorSensorData, short channel)
{
	return 0;
}

static short getMotorCurrent(X80Pro *self, const int *motorSensorData, short channel)
{
	return 0;
}

static short getEncoderDirection(X80Pro *self, const int *motorSensorData, short channel)
{
	return 0;
}

static short getEncoderPulse(X80Pro *self, const int *motorSensorData, short channel)
{
	return 0;
}

static short getEncoderSpeed(X80Pro *self, const int *motorSensorData, short channel)
{
	return 0;
}

static short getCustomAD(X80Pro *self, const int *customSensorData, short channel)
{
	return 0;
}

static short getCustomDIn(X80Pro *self, const int *customSensorData, unsigned char channel)
{
	return 0;
}

void X80Pro_init(X80Pro *self)
{
	PMS5005_init(&PMS5005);

    self->getSensorSonar = getSensorSonar;
    self->getSensorIRRange = getSensorIRRange;
    self->getSensorHumanAlarm = getSensorHumanAlarm;
    self->getSensorHumanMotion = getSensorHumanMotion;
    self->getSensorTiltingX = getSensorTiltingX;
    self->getSensorTiltingY = getSensorTiltingY;
    self->getSensorOverheat = getSensorOverheat;
    self->getSensorTemperature = getSensorTemperature;
    self->getSensorIRCode = getSensorIRCode;
    self->getSensorBatteryAD = getSensorBatteryAD;
    self->getSensorRefVoltage = getSensorRefVoltage;
    self->getSensorPotVoltage = getSensorPotVoltage;
    self->getSensorPot = getSensorPot;
    self->getMotorCurrent = getMotorCurrent;
    self->getEncoderDirection = getEncoderDirection;
    self->getEncoderPulse = getEncoderPulse;
    self->getEncoderSpeed = getEncoderSpeed;
    self->getCustomAD = getCustomAD;
    self->getCustomDIn = getCustomDIn;
}
