#pragma once
#ifndef PMS5005_H
#define PMS5005_H

#include "../buffer/Buffer.h"

static int HEADER_LENGTH = 6;
static int PAYLOAD_OFFSET = 6;
static int DID_OFFSET = 4;
static int MOTOR_SENSOR_REQUEST_SIZE = 10;
static int STANDARD_SENSOR_REQUEST_SIZE = 10;
static int CUSTOM_SENSOR_REQUEST_SIZE = 10;
static int ALL_SENSOR_REQUEST_SIZE = 10;
static int ENABLE_ALL_SENSOR_SENDING_SIZE = 9;
static int ENABLE_STANDARD_SENSOR_SENDING_SIZE = 9;
static int ENABLE_CUSTOM_SENSOR_SENDING_SIZE = 9;
static int DISABLE_MOTOR_SENSOR_SENDING_SIZE = 10;
static int DISABLE_STANDARD_SENSOR_SENDING_SIZE = 10;
static int DISABLE_ALL_SENSOR_SENDING_SIZE = 10;
static int SET_MOTOR_POLARITY_SIZE = 12;
static int ENABLE_DC_MOTOR_SIZE = 11;
static int DISABLE_DC_MOTOR_SIZE = 11;
static int RESUME_DC_MOTOR_SIZE = 11;
static int SUSPEND_DC_MOTOR_SIZE = 11;
static int SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = 20;
static int SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = 20;
static int SET_DC_MOTOR_SENSOR_FILTER_SIZE = 12;
static int SET_DC_MOTOR_CTRL_MODE_SIZE = 12;
static int DC_MOTOR_POSITION_TIME_CTRL_SIZE = 14;
static int DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 12;
static int DC_MOTOR_PWM_TIME_CTRL_SIZE = 14;
static int DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = 12;
static int DC_MOTOR_POSITION_TIME_CTRL_ALL_SIZE = 23;
static int DC_MOTOR_POSITION_NON_TIME_CTRL_ALL_SIZE = 21;
static int DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = 23;
static int DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = 21;
static int DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = 23;
static int DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = 21;
static int ENABLE_SERVO_SIZE = 11;
static int DISABLE_SERVO_SIZE = 11;
static int SERVO_TIME_CTRL_SIZE = 15;
static int SERVO_NON_TIME_CTRL_SIZE = 13;
static int SERVO_TIME_CTRL_ALL_SIZE = 23;
static int SERVO_NON_TIME_CTRL_ALL_SIZE = 21;
static unsigned char STX0 = 94;
static unsigned char STX1 = 2;
static unsigned char ETX0 = 94;
static unsigned char ETX1 = 13;
static unsigned char POSITION_CTRL_DID = 3;
static unsigned char ALL_POSITION_CTRL_DID = 4;
static unsigned char PWM_CTRL_DID = 5;
static unsigned char ALL_PWM_CTRL_DID = 6;
static unsigned char PARAM_SET_DID = 7;
static unsigned char DC_POSITION_PID_DID = 7;
static unsigned char DC_VELOCITY_PID_DID = 8;
static unsigned char DC_SENSOR_USAGE_DID = 13;
static unsigned char DC_CTRL_MODE_DID = 14;
static unsigned char POWER_CTRL_DID = 22;
static unsigned char LCD_CTRL_DID = 23;
static unsigned char VELOCITY_CTRL_DID = 26;
static unsigned char ALL_VELOCITY_CTRL_DID = 27;
static unsigned char SERVO_CTRL_DID = 28;
static unsigned char ALL_SERVO_CTRL_DID = 29;
static unsigned char TOGGLE_DC_MOTORS_DID = 30;
static unsigned char CONSTELLATION_CTRL_DID = 80;
static unsigned char GET_MOTOR_SENSOR_DATA_DID = 123;
static unsigned char GET_CUSTOM_SENSOR_DATA_DID = 124;
static unsigned char GET_STANDARD_SENSOR_DATA_DID = 125;
static unsigned char GET_ALL_SENSOR_DATA_DID = 127;
static short SETUP_COM = 255;
static unsigned char PWM_CTRL_MODE = 0;
static unsigned char POSITION_CTRL_MODE = 1;
static unsigned char VELOCITY_CTRL_MODE = 2;
static unsigned char KP_ID = 1;
static unsigned char KD_ID = 2;
static unsigned char KI_ID = 3;
static short NON_CTRL_PACKET = (short)0xffff;
static short NO_CTRL = (short)0x8000;
static int ULTRASONIC_OFFSET = 6; /* 0 + HEADER_LENGTH */
static int ENCODER_PULSE_OFFSET = 30; /* 24 + HEADER_LENGTH */
static int ENCODER_SPEED_OFFSET = 38; /* 32 + HEADER_LENGTH */
static int STANDARD_IR_RANGE_OFFSET = 30; /* 24 + HEADER_LENGTH */
static int CUSTOM_IR_RANGE_OFFSET = 10; /* 4 + HEADER_LENGTH */
static int HUMAN_ALARM_OFFSET = 12; /* 6 + HEADER_LENGTH */
static int HUMAN_MOTION_OFFSET = 14; /* 8 + HEADER_LENGTH */
static int TILTING_X_OFFSET = 20; /* 14 + HEADER_LENGTH */
static int TILTING_Y_OFFSET = 22; /* 16 + HEADER_LENGTH */
static int ENCODER_DIRECTION_OFFSET = 38; /* 32 + HEADER_LENGTH */
static int MOTOR_SPEED_OFFSET = 32; /* 26 + HEADER_LENGTH */
static int CUSTOM_AD_OFFSET = 6; /* 0 + HEADER_LENGTH */
static int TEMPERATURE_AD_OFFSET = 28; /* 22 + HEADER_LENGTH */
static int OVERHEAT_SENSOR_OFFSET = 24; /* 18 + HEADER_LENGTH */
static int IR_COMMAND_OFFSET = 32; /* 26 + HEADER_LENGTH */
static int BATTERY_SENSOR_OFFSET = 36; /* 30 + HEADER_LENGTH */
static int REFERENCE_VOLTAGE_OFFSET = 42; /* 36 + HEADER_LENGTH */
static int POTENTIOMETER_POWER_OFFSET = 44; /* 38 + HEADER_LENGTH */
static int POTENTIOMETER_SENSOR_OFFSET = 6; /* 0 + HEADER_LENGTH */
static int MOTOR_CURRENT_SENSOR_OFFSET = 18; /* 12 + HEADER_LENGTH */
static int MOTOR_SENSOR_DATA_LENGTH = 34;
static int CUSTOM_SENSOR_DATA_LENGTH = 37;
static int STANDARD_SENSOR_DATA_LENGTH = 33;

typedef struct
{
    int HEADER_LENGTH;
    int PAYLOAD_OFFSET;
    int DID_OFFSET;

    int MOTOR_SENSOR_REQUEST_SIZE;
    int STANDARD_SENSOR_REQUEST_SIZE;
    int CUSTOM_SENSOR_REQUEST_SIZE;
    int ALL_SENSOR_REQUEST_SIZE;
    int ENABLE_ALL_SENSOR_SENDING_SIZE;
    int ENABLE_STANDARD_SENSOR_SENDING_SIZE;
    int ENABLE_CUSTOM_SENSOR_SENDING_SIZE;
    int DISABLE_MOTOR_SENSOR_SENDING_SIZE;
    int DISABLE_STANDARD_SENSOR_SENDING_SIZE;
    int DISABLE_ALL_SENSOR_SENDING_SIZE;
    int SET_MOTOR_POLARITY_SIZE;
    int ENABLE_DC_MOTOR_SIZE;
    int DISABLE_DC_MOTOR_SIZE;
    int RESUME_DC_MOTOR_SIZE;
    int SUSPEND_DC_MOTOR_SIZE;
    int SET_DC_MOTOR_POSITION_CTRL_PID_SIZE;
    int SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE;
    int SET_DC_MOTOR_SENSOR_FILTER_SIZE;
    int SET_DC_MOTOR_CTRL_MODE_SIZE;
    int DC_MOTOR_POSITION_TIME_CTRL_SIZE;
    int DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE;
    int DC_MOTOR_PWM_TIME_CTRL_SIZE;
    int DC_MOTOR_PWM_NON_TIME_CTRL_SIZE;
    int DC_MOTOR_POSITION_TIME_CTRL_ALL_SIZE;
    int DC_MOTOR_POSITION_NON_TIME_CTRL_ALL_SIZE;
    int DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE;
    int DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE;
    int DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE;
    int DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE;
    int ENABLE_SERVO_SIZE;
    int DISABLE_SERVO_SIZE;
    int SERVO_TIME_CTRL_SIZE;
    int SERVO_NON_TIME_CTRL_SIZE;
    int SERVO_TIME_CTRL_ALL_SIZE;
    int SERVO_NON_TIME_CTRL_ALL_SIZE;
	
	int MOTOR_SENSOR_DATA_LENGTH;
	int CUSTOM_SENSOR_DATA_LENGTH;
	int STANDARD_SENSOR_DATA_LENGTH;

    /* Start transmission, End transmission */
    unsigned char STX0;
    unsigned char STX1;
    unsigned char ETX0;
    unsigned char ETX1;

    /* Data ID (DID) descriptor listing */
    unsigned char POSITION_CTRL_DID;
    unsigned char ALL_POSITION_CTRL_DID;
    unsigned char PWM_CTRL_DID;
    unsigned char ALL_PWM_CTRL_DID;
    unsigned char PARAM_SET_DID;

    /* SubCommands under PARAM_SET */
    unsigned char DC_POSITION_PID_DID;     /* position PID Control */
    unsigned char DC_VELOCITY_PID_DID;     /* velocity PID Control */
    unsigned char DC_SENSOR_USAGE_DID;
    unsigned char DC_CTRL_MODE_DID;

    unsigned char POWER_CTRL_DID;
    unsigned char LCD_CTRL_DID;
    unsigned char VELOCITY_CTRL_DID;
    unsigned char ALL_VELOCITY_CTRL_DID;
    unsigned char SERVO_CTRL_DID;
    unsigned char ALL_SERVO_CTRL_DID;
    unsigned char TOGGLE_DC_MOTORS_DID;
    unsigned char CONSTELLATION_CTRL_DID;
    unsigned char GET_MOTOR_SENSOR_DATA_DID;
    unsigned char GET_CUSTOM_SENSOR_DATA_DID;
    unsigned char GET_STANDARD_SENSOR_DATA_DID;
    unsigned char GET_ALL_SENSOR_DATA_DID;
    /* to use as uunsigned char: (unsigned char)(SETUP_COM & 0xff) */
    short SETUP_COM;
    /* End Data ID (DID) descriptor listing */

    unsigned char PWM_CTRL_MODE;
    unsigned char POSITION_CTRL_MODE;
    unsigned char VELOCITY_CTRL_MODE;
    
    unsigned char KP_ID; /* progressive id */
    unsigned char KD_ID; /* derivative id */
    unsigned char KI_ID; /* integral id */
    
    short NON_CTRL_PACKET; /* no ctrl command */
    short NO_CTRL;

    /* Sensor Data Offsets */
    int ULTRASONIC_OFFSET;
    int ENCODER_PULSE_OFFSET; 
    int ENCODER_SPEED_OFFSET;
    int STANDARD_IR_RANGE_OFFSET;
    int CUSTOM_IR_RANGE_OFFSET; /* CustomAD3 */
    int HUMAN_ALARM_OFFSET;
    int HUMAN_MOTION_OFFSET;
    int TILTING_X_OFFSET;
    int TILTING_Y_OFFSET;
    int ENCODER_DIRECTION_OFFSET;
    int MOTOR_SPEED_OFFSET;
    int CUSTOM_AD_OFFSET;
    int TEMPERATURE_AD_OFFSET;
    int OVERHEAT_SENSOR_OFFSET;
    int IR_COMMAND_OFFSET;
    int BATTERY_SENSOR_OFFSET;
    int REFERENCE_VOLTAGE_OFFSET;
    int POTENTIOMETER_POWER_OFFSET;
    int POTENTIOMETER_SENSOR_OFFSET;
    int MOTOR_CURRENT_SENSOR_OFFSET;

    /* methods */
	int (*test)();
    unsigned char (*calcCRC)(Buffer *buf);
    Buffer * (*motorSensorRequest)(Buffer *buf, short packetNumber);
    Buffer * (*standardSensorRequest)(Buffer *buf, short packetNumber);
    Buffer * (*customSensorRequest)(Buffer *buf, short packetNumber);
    Buffer * (*allSensorRequest)(Buffer *buf, short packetNumber);
    Buffer * (*enableMotorSensorSending)(Buffer *buf);
    Buffer * (*enableStandardSensorSending)(Buffer *buf);
    Buffer * (*enableCustomSensorSending)(Buffer *buf);
    Buffer * (*enableAllSensorSending)(Buffer *buf);
    Buffer * (*disableMotorSensorSending)(Buffer *buf);
    Buffer * (*disableStandardSensorSending)(Buffer *buf);
    Buffer * (*disableCustomSensorSending)(Buffer *buf);
    Buffer * (*disableAllSensorSending)(Buffer *buf);
    Buffer * (*setMotorSensorPeriod)(Buffer *buf, short timePeriod);
    Buffer * (*setStandardSensorPeriod)(Buffer *buf, short timePeriod);
    Buffer * (*setCustomSensorPeriod)(Buffer *buf, short timePeriod);
    Buffer * (*setAllSensorPeriod)(Buffer *buf, short timePeriod);
    Buffer * (*setIRCtrlOutput)(Buffer *buf, short loWord, short hiWord);
    Buffer * (*setCustomDOut)(Buffer *buf, unsigned char ival);
    Buffer * (*setMotorPolarity)(Buffer *buf, unsigned char channel, unsigned char polarity);
    Buffer * (*enableDCMotor)(Buffer *buf, unsigned char channel);
    Buffer * (*disableDCMotor)(Buffer *buf, unsigned char channel);
    Buffer * (*resumeDCMotor)(Buffer *buf, unsigned char channel);
    Buffer * (*suspendDCMotor)(Buffer *buf, unsigned char channel);
    Buffer * (*setDCMotorPositionPID)(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
    Buffer * (*setDCMotorVelocityPID)(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
    Buffer * (*setDCMotorSensorFilter)(Buffer *buf, unsigned char channel, short filterMethod);
    Buffer * (*setDCMotorSensorUsage)(Buffer *buf, unsigned char channel, unsigned char sensorUsage);
    Buffer * (*setDCMotorCtrlMode)(Buffer *buf, unsigned char channel, unsigned char controlMode);
    Buffer * (*setDCMotorPositionWithTime)(Buffer *buf, unsigned char channel, short pos, short timePeriod);
    Buffer * (*setDCMotorPosition)(Buffer *buf, unsigned char channel, short pos);
    Buffer * (*setDCMotorPulseWithTime)(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod);
    Buffer * (*setDCMotorPulse)(Buffer *buf, unsigned char channel, short pulseWidth);
    Buffer * (*setAllDCMotorPositionsWithTime)(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod);
    Buffer * (*setAllDCMotorPositions)(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5);
    Buffer * (*setBothDCMotorPositionsWithTime)(Buffer *buf, short posLeft, short posRight, short timePeriod);
    Buffer * (*setBothDCMotorPositions)(Buffer *buf, short posLeft, short posRight);
    Buffer * (*setAllDCMotorVelocitiesWithTime)(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod);
    Buffer * (*setAllDCMotorVelocities)(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5);
    Buffer * (*setBothDCMotorVelocitiesWithTime)(Buffer *buf, short vLeft, short vRight, short timePeriod);
    Buffer * (*setBothDCMotorVelocities)(Buffer *buf, short vLeft, short vRight);
    Buffer * (*setAllDCMotorPulsesWithTime)(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
    Buffer * (*setAllDCMotorPulses)(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
    Buffer * (*setBothDCMotorPulsesWithTime)(Buffer *buf, short pLeft, short pRight, short timePeriod);
    Buffer * (*setBothDCMotorPulses)(Buffer *buf, short pLeft, short pRight);
    Buffer * (*enableServo)(Buffer *buf, unsigned char channel);
    Buffer * (*disableServo)(Buffer *buf, unsigned char channel);
    Buffer * (*setServoPulseWithTime)(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod);
    Buffer * (*setServoPulse)(Buffer *buf, unsigned char channel, short pulseWidth);
    Buffer * (*setAllServoPulsesWithTime)(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
    Buffer * (*setAllServoPulses)(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
    Buffer * (*setBothServoPulsesWithTime)(Buffer *buf, short pLeft, short pRight, short timePeriod);
    Buffer * (*setBothServoPulses)(Buffer *buf, short pLeft, short pRight);
    Buffer * (*setLCDDisplayPMS)(Buffer *buf, char *bmpFileName);
} PMS5005_t;

extern void PMS5005_init(PMS5005_t *self);

#endif
