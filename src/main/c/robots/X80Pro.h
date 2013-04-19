#pragma once
#ifndef X80PRO_H
#define X80PRO_H

typedef struct X80Pro
{
	void (*connect)(struct X80Pro *self, char *ipaddr);
	void (*resetHead)();

	/* pms5005 methods */
    void (*motorSensorRequest)(short packetNumber);
    void (*standardSensorRequest)(short packetNumber);
    void (*customSensorRequest)(short packetNumber);
    void (*allSensorRequest)(short packetNumber);
    void (*enableMotorSensorSending)(Buffer *buf);
    void (*enableStandardSensorSending)(Buffer *buf);
    void (*enableCustomSensorSending)(Buffer *buf);
    void (*enableAllSensorSending)(Buffer *buf);
    void (*disableMotorSensorSending)(Buffer *buf);
    void (*disableStandardSensorSending)(Buffer *buf);
    void (*disableCustomSensorSending)(Buffer *buf);
    void (*disableAllSensorSending)(Buffer *buf);
    void (*setMotorSensorPeriod)(short timePeriod);
    void (*setStandardSensorPeriod)(short timePeriod);
    void (*setCustomSensorPeriod)(short timePeriod);
    void (*setAllSensorPeriod)(short timePeriod);
    void (*setIRCtrlOutput)(short loWord, short hiWord);
    void (*setCustomDOut)(unsigned char ival);
    void (*setMotorPolarity)(unsigned char channel, unsigned char polarity);
    void (*enableDCMotor)(unsigned char channel);
    void (*disableDCMotor)(unsigned char channel);
    void (*resumeDCMotor)(unsigned char channel);
    void (*suspendDCMotor)(unsigned char channel);
    void (*setDCMotorPositionPID)(unsigned char channel, short kp, short kd, short ki_x100);
    void (*setDCMotorVelocityPID)(unsigned char channel, short kp, short kd, short ki_x100);
    void (*setDCMotorSensorFilter)(unsigned char channel, short filterMethod);
    void (*setDCMotorSensorUsage)(unsigned char channel, unsigned char sensorUsage);
    void (*setDCMotorCtrlMode)(unsigned char channel, unsigned char controlMode);
    void (*setDCMotorPositionWithTime)(unsigned char channel, short pos, short timePeriod);
    void (*setDCMotorPosition)(unsigned char channel, short pos);
    void (*setDCMotorPulseWithTime)(unsigned char channel, short pulseWidth, short timePeriod);
    void (*setDCMotorPulse)(unsigned char channel, short pulseWidth);
    void (*setAllDCMotorPositionsWithTime)(short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod);
    void (*setAllDCMotorPositions)(short pos0, short pos1, short pos2, short pos3, short pos4, short pos5);
    void (*setBothDCMotorPositionsWithTime)(short posLeft, short posRight, short timePeriod);
    void (*setBothDCMotorPositions)(short posLeft, short posRight);
    void (*setAllDCMotorVelocitiesWithTime)(short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod);
    void (*setAllDCMotorVelocities)(short v0, short v1, short v2, short v3, short v4, short v5);
    void (*setBothDCMotorVelocitiesWithTime)(short vLeft, short vRight, short timePeriod);
    void (*setBothDCMotorVelocities)(short vLeft, short vRight);
    void (*setAllDCMotorPulsesWithTime)(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
    void (*setAllDCMotorPulses)(short p0, short p1, short p2, short p3, short p4, short p5);
    void (*setBothDCMotorPulsesWithTime)(short pLeft, short pRight, short timePeriod);
    void (*setBothDCMotorPulses)(short pLeft, short pRight);
    void (*enableServo)(unsigned char channel);
    void (*disableServo)(unsigned char channel);
    void (*setServoPulseWithTime)(unsigned char channel, short pulseWidth, short timePeriod);
    void (*setServoPulse)(unsigned char channel, short pulseWidth);
    void (*setAllServoPulsesWithTime)(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
    void (*setAllServoPulses)(short p0, short p1, short p2, short p3, short p4, short p5);
    void (*setBothServoPulsesWithTime)(short pLeft, short pRight, short timePeriod);
    void (*setBothServoPulses)(short pLeft, short pRight);
    void (*setLCDDisplayPMS)(char *bmpFileName);
	
	/* get methods */
    short (*getSensorSonar)(struct X80Pro *self, const int *standardSensorData, short channel);
    short (*getSensorIRRange)(struct X80Pro *self, const int *standardSensorData, const int *customSensorData, short channel);
    short (*getSensorHumanAlarm)(struct X80Pro *self, const int *standardSensorData, short channel);
    short (*getSensorHumanMotion)(struct X80Pro *self, const int *standardSensorData, short channel);
    short (*getSensorTiltingX)(struct X80Pro *self, const int *standardSensorData);
    short (*getSensorTiltingY)(struct X80Pro *self, const int *standardSensorData);
    short (*getSensorOverheat)(struct X80Pro *self, const int *standardSensorData, short channel);
	short (*getSensorTemperature)(struct X80Pro *self, const int *standardSensorData);
    short (*getSensorIRCode)(struct X80Pro *self, const int *standardSensorData, short channel);
    short (*getSensorBatteryAD)(struct X80Pro *self, const int *standardSensorData, short channel);
    short (*getSensorRefVoltage)(struct X80Pro *self, const int *standardSensorData);
    short (*getSensorPotVoltage)(struct X80Pro *self, const int *standardSensorData);
    short (*getSensorPot)(struct X80Pro *self, const int *motorSensorData, short channel);
    short (*getMotorCurrent)(struct X80Pro *self, const int *motorSensorData, short channel);
    short (*getEncoderDirection)(struct X80Pro *self, const int *motorSensorData, short channel);
    short (*getEncoderPulse)(struct X80Pro *self, const int *motorSensorData, short channel);
    short (*getEncoderSpeed)(struct X80Pro *self, const int *motorSensorData, short channel);
    short (*getCustomAD)(struct X80Pro *self, const int *customSensorData, short channel);
    short (*getCustomDIn)(struct X80Pro *self, const int *customSensorData, unsigned char channel);

} X80Pro;

extern void X80Pro_init(X80Pro *self);

#endif
