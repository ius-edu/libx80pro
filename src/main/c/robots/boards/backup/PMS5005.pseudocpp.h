#pragma once
#ifndef PMS5005_H
#define PMS5005_H

#include "../buffer/Buffer.h"

static short HEADER_LENGTH = 7;
static short PAYLOAD_OFFSET = 6;
static short DID_OFFSET = 4;
static short MOTOR_SENSOR_REQUEST_SIZE = 10;
static short STANDARD_SENSOR_REQUEST_SIZE = 10;
static short CUSTOM_SENSOR_REQUEST_SIZE = 10;
static short ALL_SENSOR_REQUEST_SIZE = 10;
static short ENABLE_ALL_SENSOR_SENDING_SIZE = 9;
static short ENABLE_STANDARD_SENSOR_SENDING_SIZE = 9;
static short ENABLE_CUSTOM_SENSOR_SENDING_SIZE = 9;
static short DISABLE_MOTOR_SENSOR_SENDING_SIZE = 10;
static short DISABLE_STANDARD_SENSOR_SENDING_SIZE = 10;
static short DISABLE_ALL_SENSOR_SENDING_SIZE = 10;
static short SET_MOTOR_POLARITY_SIZE = 12;
static short ENABLE_DC_MOTOR_SIZE = 11;
static short DISABLE_DC_MOTOR_SIZE = 11;
static short RESUME_DC_MOTOR_SIZE = 11;
static short SUSPEND_DC_MOTOR_SIZE = 11;
static short SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = 20;
static short SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = 20;
static short SET_DC_MOTOR_SENSOR_FILTER_SIZE = 12;
static short SET_DC_MOTOR_CTRL_MODE_SIZE = 12;
static short DC_MOTOR_POSITION_TIME_CTRL_SIZE = 14;
static short DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 12;
static short DC_MOTOR_PWM_TIME_CTRL_SIZE = 14;
static short DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = 12;
static short DC_MOTOR_POSITION_TIME_CTRL_ALL_SIZE = 23;
static short DC_MOTOR_POSITION_NON_TIME_CTRL_ALL_SIZE = 21;
static short DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = 23;
static short DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = 21;
static short DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = 23;
static short DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = 21;
static short ENABLE_SERVO_SIZE = 11;
static short DISABLE_SERVO_SIZE = 11;
static short SERVO_TIME_CTRL_SIZE = 15;
static short SERVO_NON_TIME_CTRL_SIZE = 13;
static short SERVO_TIME_CTRL_ALL_SIZE = 23;
static short SERVO_NON_TIME_CTRL_ALL_SIZE = 21;
static short STX0 = 94;
static short STX1 = 2;
static short ETX0 = 94;
static short ETX1 = 13;
static short POSITION_CTRL_DID = 3;
static short ALL_POSITION_CTRL_DID = 4;
static short PWM_CTRL_DID = 5;
static short ALL_PWM_CTRL_DID = 6;
static short PARAM_SET_DID = 7;
static short DC_POSITION_PID_DID = 7;
static short DC_VELOCITY_PID_DID = 8;
static short DC_SENSOR_USAGE_DID = 13;
static short DC_CTRL_MODE_DID = 14;
static short POWER_CTRL_DID = 22;
static short LCD_CTRL_DID = 23;
static short VELOCITY_CTRL_DID = 26;
static short ALL_VELOCITY_CTRL_DID = 27;
static short SERVO_CTRL_DID = 28;
static short ALL_SERVO_CTRL_DID = 29;
static short TOGGLE_DC_MOTORS_DID = 30;
static short CONSTELLATION_CTRL_DID = 80;
static short GET_MOTOR_SENSOR_DATA_DID = 123;
static short GET_CUSTOM_SENSOR_DATA_DID = 124;
static short GET_STANDARD_SENSOR_DATA_DID = 125;
static short GET_ALL_SENSOR_DATA_DID = 127;
static short SETUP_COM = 255;
static short PWM_CTRL_MODE = 0;
static short POSITION_CTRL_MODE = 1;
static short VELOCITY_CTRL_MODE = 2;
static short KP_ID = 1;
static short KD_ID = 2;
static short KI_ID = 3;
static short NON_CTRL_PACKET = (short)0xffff;
static short NO_CTRL = (short)0x8000;
static short ULTRASONIC_OFFSET = 7; /* 0 + HEADER_LENGTH */
static short ENCODER_PULSE_OFFSET = 31; /* 24 + HEADER_LENGTH */
static short ENCODER_SPEED_OFFSET = 39; /* 32 + HEADER_LENGTH */
static short STANDARD_IR_RANGE_OFFSET = 31; /* 24 + HEADER_LENGTH */
static short CUSTOM_IR_RANGE_OFFSET = 11; /* 4 + HEADER_LENGTH */
static short HUMAN_ALARM_OFFSET = 13; /* 6 + HEADER_LENGTH */
static short HUMAN_MOTION_OFFSET = 15; /* 8 + HEADER_LENGTH */
static short TILTING_X_OFFSET = 21; /* 14 + HEADER_LENGTH */
static short TILTING_Y_OFFSET = 23; /* 16 + HEADER_LENGTH */
static short ENCODER_DIRECTION_OFFSET = 39; /* 32 + HEADER_LENGTH */
static short MOTOR_SPEED_OFFSET = 33; /* 26 + HEADER_LENGTH */
static short CUSTOM_AD_OFFSET = 7; /* 0 + HEADER_LENGTH */
static short TEMPERATURE_AD_OFFSET = 29; /* 22 + HEADER_LENGTH */
static short OVERHEAT_SENSOR_OFFSET = 25; /* 18 + HEADER_LENGTH */
static short IR_COMMAND_OFFSET = 33; /* 26 + HEADER_LENGTH */
static short BATTERY_SENSOR_OFFSET = 37; /* 30 + HEADER_LENGTH */
static short REFERENCE_VOLTAGE_OFFSET = 43; /* 36 + HEADER_LENGTH */
static short POTENTIOMETER_POWER_OFFSET = 45; /* 38 + HEADER_LENGTH */
static short POTENTIOMETER_SENSOR_OFFSET = 7; /* 0 + HEADER_LENGTH */
static short MOTOR_CURRENT_SENSOR_OFFSET = 19; /* 12 + HEADER_LENGTH */

static const unsigned char calcCRC(Buffer *buf);
static const Buffer * motorSensorRequest(Buffer *buf, short packetNumber);
static const Buffer * standardSensorRequest(Buffer *buf, short packetNumber);
static const Buffer * customSensorRequest(Buffer *buf, short packetNumber);
static const Buffer * allSensorRequest(Buffer *buf, short packetNumber);
static const Buffer * enableMotorSensorSending(Buffer *buf);
static const Buffer * enableStandardSensorSending(Buffer *buf);
static const Buffer * enableCustomSensorSending(Buffer *buf);
static const Buffer * enableAllSensorSending(Buffer *buf);
static const Buffer * disableMotorSensorSending(Buffer *buf);
static const Buffer * disableStandardSensorSending(Buffer *buf);
static const Buffer * disableCustomSensorSending(Buffer *buf);
static const Buffer * disableAllSensorSending(Buffer *buf);
static const Buffer * setMotorSensorPeriod(Buffer *buf, short timePeriod);
static const Buffer * setStandardSensorPeriod(Buffer *buf, short timePeriod);
static const Buffer * setCustomSensorPeriod(Buffer *buf, short timePeriod);
static const Buffer * setAllSensorPeriod(Buffer *buf, short timePeriod);
static const Buffer * setIRCtrlOutput(Buffer *buf, short loWord, short hiWord);
static const Buffer * setCustomDOut(Buffer *buf, unsigned char ival);
static const Buffer * setMotorPolarity(Buffer *buf, unsigned char channel, unsigned char polarity);
static const Buffer * enableDCMotor(Buffer *buf, unsigned char channel);
static const Buffer * disableDCMotor(Buffer *buf, unsigned char channel);
static const Buffer * resumeDCMotor(Buffer *buf, unsigned char channel);
static const Buffer * suspendDCMotor(Buffer *buf, unsigned char channel);
static const Buffer * setDCMotorPositionPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
static const Buffer * setDCMotorVelocityPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
static const Buffer * setDCMotorSensorFilter(Buffer *buf, unsigned char channel, short filterMethod);
static const Buffer * setDCMotorSensorUsage(Buffer *buf, unsigned char channel, unsigned char sensorType);
static const Buffer * setDCMotorCtrlMode(Buffer *buf, unsigned char channel, unsigned char controlMode);
static const Buffer * setDCMotorPositionWithTime(Buffer *buf, unsigned char channel, short pos, short timePeriod);
static const Buffer * setDCMotorPosition(Buffer *buf, unsigned char channel, short pos);
static const Buffer * setDCMotorPulseWithTime(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod);
static const Buffer * setDCMotorPulse(Buffer *buf, unsigned char channel, short pulseWidth);
static const Buffer * setAllDCMotorPositionsWithTime(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod);
static const Buffer * setAllDCMotorPositions(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5);
static const Buffer * setBothDCMotorPositionsWithTime(Buffer *buf, short posLeft, short posRight, short timePeriod);
static const Buffer * setBothDCMotorPositions(Buffer *buf, short posLeft, short posRight);
static const Buffer * setAllDCMotorVelocitiesWithTime(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod);
static const Buffer * setAllDCMotorVelocities(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5);
static const Buffer * setBothDCMotorVelocitiesWithTime(Buffer *buf, short vLeft, short vRight, short timePeriod);
static const Buffer * setBothDCMotorVelocities(Buffer *buf, short vLeft, short vRight);
static const Buffer * setAllDCMotorPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
static const Buffer * setAllDCMotorPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
static const Buffer * setBothDCMotorPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod);
static const Buffer * setBothDCMotorPulses(Buffer *buf, short pLeft, short pRight);
static const Buffer * enableServo(Buffer *buf, unsigned char channel);
static const Buffer * disableServo(Buffer *buf, unsigned char channel);
static const Buffer * setServoPulseWithTime(Buffer *buf, unsigned char channel, short pulse, short timePeriod);
static const Buffer * setServoPulse(Buffer *buf, unsigned char channel, short pulse);
static const Buffer * setAllServoPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
static const Buffer * setAllServoPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
static const Buffer * setBothServoPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod);
static const Buffer * setBothServoPulses(Buffer *buf, short pLeft, short pRight);
static const Buffer * setLCDDisplayPMS(Buffer *buf, char *bmpFileName);

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
    const unsigned char (*calcCRC)(Buffer *);
    const Buffer * (*motorSensorRequest)(Buffer *, short);
    const Buffer * (*standardSensorRequest)(Buffer *, short);
    const Buffer * (*customSensorRequest)(Buffer *, short);
    const Buffer * (*allSensorRequest)(Buffer *, short);
    const Buffer * (*enableMotorSensorSending)(Buffer *);
    const Buffer * (*enableStandardSensorSending)(Buffer *);
    const Buffer * (*enableCustomSensorSending)(Buffer *);
    const Buffer * (*enableAllSensorSending)(Buffer *);
    const Buffer * (*disableMotorSensorSending)(Buffer *);
    const Buffer * (*disableStandardSensorSending)(Buffer *);
    const Buffer * (*disableCustomSensorSending)(Buffer *);
    const Buffer * (*disableAllSensorSending)(Buffer *);
    const Buffer * (*setMotorSensorPeriod)(Buffer *, short);
    const Buffer * (*setStandardSensorPeriod)(Buffer *, short);
    const Buffer * (*setCustomSensorPeriod)(Buffer *, short);
    const Buffer * (*setAllSensorPeriod)(Buffer *, short);
    const Buffer * (*setIRCtrlOutput)(Buffer *, short, short);
    const Buffer * (*setCustomDOut)(Buffer *, unsigned char);
    const Buffer * (*setMotorPolarity)(Buffer *, unsigned char, unsigned char);
    const Buffer * (*enableDCMotor)(Buffer *, unsigned char);
    const Buffer * (*disableDCMotor)(Buffer *, unsigned char);
    const Buffer * (*resumeDCMotor)(Buffer *, unsigned char);
    const Buffer * (*suspendDCMotor)(Buffer *, unsigned char);
    const Buffer * (*setDCMotorPositionPID)(Buffer *, unsigned char, short, short, short);
    const Buffer * (*setDCMotorVelocityPID)(Buffer *, unsigned char, short, short, short);
    const Buffer * (*setDCMotorSensorFilter)(Buffer *, unsigned char, short);
    const Buffer * (*setDCMotorSensorUsage)(Buffer *, unsigned char, unsigned char);
    const Buffer * (*setDCMotorCtrlMode)(Buffer *, unsigned char, unsigned char);
    const Buffer * (*setDCMotorPositionWithTime)(Buffer *, unsigned char, short, short);
    const Buffer * (*setDCMotorPosition)(Buffer *, unsigned char, short);
    const Buffer * (*setDCMotorPulseWithTime)(Buffer *, unsigned char, short, short);
    const Buffer * (*setDCMotorPulse)(Buffer *, unsigned char, short);
    const Buffer * (*setAllDCMotorPositionsWithTime)(Buffer *, short, short, short, short, short, short, short);
    const Buffer * (*setAllDCMotorPositions)(Buffer *, short, short, short, short, short, short);
    const Buffer * (*setBothDCMotorPositionsWithTime)(Buffer *, short, short, short);
    const Buffer * (*setBothDCMotorPositions)(Buffer *, short, short);
    const Buffer * (*setAllDCMotorVelocitiesWithTime)(Buffer *, short, short, short, short, short, short, short);
    const Buffer * (*setAllDCMotorVelocities)(Buffer *, short, short, short, short, short, short);
    const Buffer * (*setBothDCMotorVelocitiesWithTime)(Buffer *, short, short, short);
    const Buffer * (*setBothDCMotorVelocities)(Buffer *, short, short);
    const Buffer * (*setAllDCMotorPulsesWithTime)(Buffer *, short, short, short, short, short, short, short);
    const Buffer * (*setAllDCMotorPulses)(Buffer *, short, short, short, short, short, short);
    const Buffer * (*setBothDCMotorPulsesWithTime)(Buffer *, short, short, short);
    const Buffer * (*setBothDCMotorPulses)(Buffer *, short, short);
    const Buffer * (*enableServo)(Buffer *, unsigned char);
    const Buffer * (*disableServo)(Buffer *, unsigned char);
    const Buffer * (*setServoPulseWithTime)(Buffer *, unsigned char, short, short);
    const Buffer * (*setServoPulse)(Buffer *, unsigned char, short);
    const Buffer * (*setAllServoPulsesWithTime)(Buffer *, short, short, short, short, short, short, short);
    const Buffer * (*setAllServoPulses)(Buffer *, short, short, short, short, short, short);
    const Buffer * (*setBothServoPulsesWithTime)(Buffer *, short, short, short);
    const Buffer * (*setBothServoPulses)(Buffer *, short, short);
    const Buffer * (*setLCDDisplayPMS)(Buffer *, char *);
} PMS5005_t;

void PMS5005_init(PMS5005_t *self);

#endif
