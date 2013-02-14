#pragma once
#ifndef H
#define H

#include "../Buffer.h"

static int HEADER_LENGTH = 7;

static unsigned char calcCRC(Buffer buf);
static Buffer motorSensorRequest(short packetNumber);
static Buffer standardSensorRequest(short packetNumber);
static Buffer customSensorRequest(short packetNumber);
static Buffer allSensorRequest(short packetNumber);
static Buffer enableMotorSensorSending();
static Buffer enableStandardSensorSending();
static Buffer enableCustomSensorSending();
static Buffer enableAllSensorSending();
static Buffer disableMotorSensorSending();
static Buffer disableStandardSensorSending();
static Buffer disableCustomSensorSending();
static Buffer disableAllSensorSending();
static Buffer setMotorSensorPeriod(short timePeriod);
static Buffer setStandardSensorPeriod(short timePeriod);
static Buffer setCustomSensorPeriod(short timePeriod);
static Buffer setAllSensorPeriod(short timePeriod);
static short getSensorSonar(short channel, const int standardSensorAry[]);
static short getSensorIRRange(short channel, const int standardSensorAry[], const int customSensorAry[]);
static short getSensorHumanAlarm(short channel, const int standardSensorAry[]);
static short getSensorHumanMotion(short channel, const int standardSensorAry[]);
static short getSensorTiltingX(const int standardSensorAry[]);
static short getSensorTiltingY(const int standardSensorAry[]);
static short getSensorOverheat(short channel, const int standardSensorAry[]);
static short getSensorTemperature(const int standardSensorAry[]);
static short getSensorIRCode(short index, const int standardSensorAry[]);
static Buffer setIRCtrlOutput(short loWord, short hiWord);
static short getSensorBatteryAD(short channel, const int standardSensorAry[]);
static short getSensorRefVoltage(const int standardSensorAry[]);
static short getSensorPotVoltage(const int standardSensorAry[]);
static short getSensorPot(short channel, const int motorSensorAry[]);
static short getMotorCurrent(short channel, const int motorSensorAry[]);
static short getEncoderDirection(short channel, const int motorSensorAry[]);
static short getEncoderPulse(short channel, const int motorSensorAry[]);
static short getEncoderSpeed(short channel, const int motorSensorAry[]);
static short getCustomAD(short channel, const int customSensorAry[]);
static short getCustomDIn(unsigned char channel, const int customSensorAry[]);
static Buffer setCustomDOut(unsigned char ival);
static Buffer setMotorPolarity(unsigned char channel, unsigned char polarity);
static Buffer enableDCMotor(unsigned char channel);
static Buffer disableDCMotor(unsigned char channel);
static Buffer resumeDCMotor(unsigned char channel);
static Buffer suspendDCMotor(unsigned char channel);
static Buffer setDCMotorPositionPID(unsigned char channel, short kp, short kd, short ki_x100);
static Buffer setDCMotorVelocityPID(unsigned char channel, short kp, short kd, short ki_x100);
static Buffer setDCMotorSensorFilter(unsigned char channel, short filterMethod);
static Buffer setDCMotorSensorUsage(unsigned char channel, unsigned char sensorType);
static Buffer setDCMotorCtrlMode(unsigned char channel, unsigned char controlMode);
static Buffer setDCMotorPositionWithTime(unsigned char channel, short pos, short timePeriod);
static Buffer setDCMotorPosition(unsigned char channel, short pos);
static Buffer setDCMotorPulseWithTime(unsigned char channel, short pulseWidth, short timePeriod);
static Buffer setDCMotorPulse(unsigned char channel, short pulseWidth);
static Buffer setAllDCMotorPositionsWithTime(short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod);
static Buffer setAllDCMotorPositions(short pos0, short pos1, short pos2, short pos3, short pos4, short pos5);
static Buffer setBothDCMotorPositionsWithTime(short posLeft, short posRight, short timePeriod);
static Buffer setBothDCMotorPositions(short posLeft, short posRight);
static Buffer setAllDCMotorVelocitiesWithTime(short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod);
static Buffer setAllDCMotorVelocities(short v0, short v1, short v2, short v3, short v4, short v5);
static Buffer setBothDCMotorVelocitiesWithTime(short vLeft, short vRight, short timePeriod);
static Buffer setBothDCMotorVelocities(short vLeft, short vRight);
static Buffer setAllDCMotorPulsesWithTime(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
static Buffer setAllDCMotorPulses(short p0, short p1, short p2, short p3, short p4, short p5);
static Buffer setBothDCMotorPulsesWithTime(short pLeft, short pRight, short timePeriod);
static Buffer setBothDCMotorPulses(short pLeft, short pRight);
static Buffer enableServo(unsigned char channel);
static Buffer disableServo(unsigned char channel);
static Buffer setServoPulseWithTime(unsigned char channel, short pulse, short timePeriod);
static Buffer setServoPulse(unsigned char channel, short pulse);
static Buffer setAllServoPulsesWithTime(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
static Buffer setAllServoPulses(short p0, short p1, short p2, short p3, short p4, short p5);
static Buffer setBothServoPulsesWithTime(short pLeft, short pRight, short timePeriod);
static Buffer setBothServoPulses(short pLeft, short pRight);
static Buffer setLCDDisplayPMS(char *bmpFileName);

struct PMS5005_t
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
    unsigned char DC_POSITION_PID_DID;     /* positon PID Control */
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
    unsigned char (*calcCRC)();
    Buffer (*motorSensorRequest)(short);
    Buffer (*standardSensorRequest)(short);
    Buffer (*customSensorRequest)(short);
    Buffer (*allSensorRequest)(short);
    Buffer (*enableMotorSensorSending)(struct PMS5005_t);
    Buffer (*enableStandardSensorSending)(struct PMS5005_t);
    Buffer (*enableCustomSensorSending)(struct PMS5005_t);
    Buffer (*enableAllSensorSending)(struct PMS5005_t);
    Buffer (*disableMotorSensorSending)(struct PMS5005_t);
    Buffer (*disableStandardSensorSending)(struct PMS5005_t);
    Buffer (*disableCustomSensorSending)(struct PMS5005_t);
    Buffer (*disableAllSensorSending)(struct PMS5005_t);
    Buffer (*setMotorSensorPeriod)(short);
    Buffer (*setStandardSensorPeriod)(short);
    Buffer (*setCustomSensorPeriod)(short);
    Buffer (*setAllSensorPeriod)(short);
    short (*getSensorSonar)(short, const int[]);
    short (*getSensorIRRange)(short, const int[], const int[]);
    short (*getSensorHumanAlarm)(short, const int[]);
    short (*getSensorHumanMotion)(short, const int[]);
    short (*getSensorTiltingX)(const int[]);
    short (*getSensorTiltingY)(const int[]);
    short (*getSensorOverheat)(short, const int[]);
    short (*getSensorTemperature)(const int[]);
    short (*getSensorIRCode)(short, const int[]);
    Buffer (*setIRCtrlOutput)(short, short);
    short (*getSensorBatteryAD)(short, const int[]);
    short (*getSensorRefVoltage)(const int[]);
    short (*getSensorPotVoltage)(const int[]);
    short (*getSensorPot)(short, const int[]);
    short (*getMotorCurrent)(short, const int[]);
    short (*getEncoderDirection)(short, const int[]);
    short (*getEncoderPulse)(short, const int[]);
    short (*getEncoderSpeed)(short, const int[]);
    short (*getCustomAD)(short, const int[]);
    short (*getCustomDIn)(unsigned char, const int[]);
    Buffer (*setCustomDOut)(unsigned char);
    Buffer (*setMotorPolarity)(unsigned char, unsigned char);
    Buffer (*enableDCMotor)(unsigned char);
    Buffer (*disableDCMotor)(unsigned char);
    Buffer (*resumeDCMotor)(unsigned char);
    Buffer (*suspendDCMotor)(unsigned char);
    Buffer (*setDCMotorPositionPID)(unsigned char, short, short, short);
    Buffer (*setDCMotorVelocityPID)(unsigned char, short, short, short);
    Buffer (*setDCMotorSensorFilter)(unsigned char, short);
    Buffer (*setDCMotorSensorUsage)(unsigned char, unsigned char);
    Buffer (*setDCMotorCtrlMode)(unsigned char, unsigned char);
    Buffer (*setDCMotorPositionWithTime)(unsigned char, short, short);
    Buffer (*setDCMotorPosition)(unsigned char, short);
    Buffer (*setDCMotorPulseWithTime)(unsigned char, short, short);
    Buffer (*setDCMotorPulse)(unsigned char, short);
    Buffer (*setAllDCMotorPositionsWithTime)(short, short, short, short, short, short, short);
    Buffer (*setAllDCMotorPositions)(short, short, short, short, short, short);
    Buffer (*setBothDCMotorPositionsWithTime)(short, short, short);
    Buffer (*setBothDCMotorPositions)(short, short);
    Buffer (*setAllDCMotorVelocitiesWithTime)(short, short, short, short, short, short, short);
    Buffer (*setAllDCMotorVelocities)(short, short, short, short, short, short);
    Buffer (*setBothDCMotorVelocitiesWithTime)(short, short, short);
    Buffer (*setBothDCMotorVelocities)(short, short);
    Buffer (*setAllDCMotorPulsesWithTime)(short, short, short, short, short, short, short);
    Buffer (*setAllDCMotorPulses)(short, short, short, short, short, short);
    Buffer (*setBothDCMotorPulsesWithTime)(short, short, short);
    Buffer (*setBothDCMotorPulses)(short, short);
    Buffer (*enableServo)(unsigned char);
    Buffer (*disableServo)(unsigned char);
    Buffer (*setServoPulseWithTime)(unsigned char, short, short);
    Buffer (*setServoPulse)(unsigned char, short);
    Buffer (*setAllServoPulsesWithTime)(short, short, short, short, short, short, short);
    Buffer (*setAllServoPulses)(short, short, short, short, short, short);
    Buffer (*setBothServoPulsesWithTime)(short, short, short);
    Buffer (*setBothServoPulses)(short, short);
    Buffer (*setLCDDisplayPMS)(char*);
} PMS5005 = {
    .HEADER_LENGTH = HEADER_LENGTH,
    .PAYLOAD_OFFSET = 6,
    .DID_OFFSET = 4,
    .MOTOR_SENSOR_REQUEST_SIZE = 10,
    .STANDARD_SENSOR_REQUEST_SIZE = 10,
    .CUSTOM_SENSOR_REQUEST_SIZE = 10,
    .ALL_SENSOR_REQUEST_SIZE = 10,
    .ENABLE_ALL_SENSOR_SENDING_SIZE = 9,
    .ENABLE_STANDARD_SENSOR_SENDING_SIZE = 9,
    .ENABLE_CUSTOM_SENSOR_SENDING_SIZE = 9,
    .DISABLE_MOTOR_SENSOR_SENDING_SIZE = 10,
    .DISABLE_STANDARD_SENSOR_SENDING_SIZE = 10,
    .DISABLE_ALL_SENSOR_SENDING_SIZE = 10,
    .SET_MOTOR_POLARITY_SIZE = 12,
    .ENABLE_DC_MOTOR_SIZE = 11,
    .DISABLE_DC_MOTOR_SIZE = 11,
    .RESUME_DC_MOTOR_SIZE = 11,
    .SUSPEND_DC_MOTOR_SIZE = 11,
    .SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = 20,
    .SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = 20,
    .SET_DC_MOTOR_SENSOR_FILTER_SIZE = 12,
    .SET_DC_MOTOR_CTRL_MODE_SIZE = 12,
    .DC_MOTOR_POSITION_TIME_CTRL_SIZE = 14,
    .DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 12,
    .DC_MOTOR_PWM_TIME_CTRL_SIZE = 14,
    .DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = 12,
    .DC_MOTOR_POSITION_TIME_CTRL_SIZE = 23,
    .DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 21,
    .DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = 23,
    .DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = 21,
    .DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = 23,
    .DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = 21,
    .ENABLE_SERVO_SIZE = 11,
    .DISABLE_SERVO_SIZE = 11,
    .SERVO_TIME_CTRL_SIZE = 15,
    .SERVO_NON_TIME_CTRL_SIZE = 13,
    .SERVO_TIME_CTRL_ALL_SIZE = 23,
    .SERVO_NON_TIME_CTRL_ALL_SIZE = 21,
    .STX0 = 94,
    .STX1 = 2,
    .ETX0 = 94,
    .ETX1 = 13,
    .POSITION_CTRL_DID = 3,
    .ALL_POSITION_CTRL_DID = 4,
    .PWM_CTRL_DID = 5,
    .ALL_PWM_CTRL_DID = 6,
    .PARAM_SET_DID = 7,
    .DC_POSITION_PID_DID = 7,
    .DC_VELOCITY_PID_DID = 8,
    .DC_SENSOR_USAGE_DID = 13,
    .DC_CTRL_MODE_DID = 14,
    .POWER_CTRL_DID = 22,
    .LCD_CTRL_DID = 23,
    .VELOCITY_CTRL_DID = 26,
    .ALL_VELOCITY_CTRL_DID = 27,
    .SERVO_CTRL_DID = 28,
    .ALL_SERVO_CTRL_DID = 29,
    .TOGGLE_DC_MOTORS_DID = 30,
    .CONSTELLATION_CTRL_DID = 80,
    .GET_MOTOR_SENSOR_DATA_DID = 123,
    .GET_CUSTOM_SENSOR_DATA_DID = 124,
    .GET_STANDARD_SENSOR_DATA_DID = 125,
    .GET_ALL_SENSOR_DATA_DID = 127,
    .SETUP_COM = 255,
    .PWM_CTRL_MODE = 0,
    .POSITION_CTRL_MODE = 1,
    .VELOCITY_CTRL_MODE = 2,
    .KP_ID = 1,
    .KD_ID = 2,
    .KI_ID = 3,
    .NON_CTRL_PACKET = 0xffff,
    .NO_CTRL = 0x8000,
    .ULTRASONIC_OFFSET = 0 + HEADER_LENGTH,
    .ENCODER_PULSE_OFFSET = 24 + HEADER_LENGTH,
    .ENCODER_SPEED_OFFSET = 32 + HEADER_LENGTH,
    .STANDARD_IR_RANGE_OFFSET = 24 + HEADER_LENGTH,
    .CUSTOM_IR_RANGE_OFFSET = 4 + HEADER_LENGTH,
    .HUMAN_ALARM_OFFSET = 6 + HEADER_LENGTH,
    .HUMAN_MOTION_OFFSET = 8 + HEADER_LENGTH,
    .TILTING_X_OFFSET = 14 + HEADER_LENGTH,
    .TILTING_Y_OFFSET = 16 + HEADER_LENGTH,
    .ENCODER_DIRECTION_OFFSET = 32 + HEADER_LENGTH,
    .MOTOR_SPEED_OFFSET = 26 + HEADER_LENGTH,
    .CUSTOM_AD_OFFSET = 0 + HEADER_LENGTH,
    .TEMPERATURE_AD_OFFSET = 22 + HEADER_LENGTH,
    .OVERHEAT_SENSOR_OFFSET = 18 + HEADER_LENGTH,
    .IR_COMMAND_OFFSET = 26 + HEADER_LENGTH,
    .BATTERY_SENSOR_OFFSET = 30 + HEADER_LENGTH,
    .REFERENCE_VOLTAGE_OFFSET = 36 + HEADER_LENGTH,
    .POTENTIOMETER_POWER_OFFSET = 38 + HEADER_LENGTH,
    .POTENTIOMETER_SENSOR_OFFSET = 0 + HEADER_LENGTH,
    .MOTOR_CURRENT_SENSOR_OFFSET = 12 + HEADER_LENGTH,
    
    /* methods */
    .calcCRC = calcCRC,
    .motorSensorRequest = motorSensorRequest,
    .standardSensorRequest = standardSensorRequest,
    .customSensorRequest = customSensorRequest,
    .allSensorRequest = allSensorRequest,
    .enableMotorSensorSending = enableMotorSensorSending,
    .enableStandardSensorSending = enableStandardSensorSending,
    .enableCustomSensorSending = enableCustomSensorSending,
    .enableAllSensorSending = enableAllSensorSending,
    .disableMotorSensorSending = disableMotorSensorSending,
    .disableStandardSensorSending = disableStandardSensorSending,
    .disableCustomSensorSending = disableCustomSensorSending,
    .disableAllSensorSending = disableAllSensorSending,
    .setMotorSensorPeriod = setMotorSensorPeriod,
    .setStandardSensorPeriod = setStandardSensorPeriod,
    .setCustomSensorPeriod = setCustomSensorPeriod,
    .setAllSensorPeriod = setAllSensorPeriod,
    .getSensorSonar = getSensorSonar,
    .getSensorIRRange = getSensorIRRange,
    .getSensorHumanAlarm = getSensorHumanAlarm,
    .getSensorHumanMotion = getSensorHumanMotion,
    .getSensorTiltingX = getSensorTiltingX,
    .getSensorTiltingY = getSensorTiltingY,
    .getSensorOverheat = getSensorOverheat,
    .getSensorTemperature = getSensorTemperature,
    .getSensorIRCode = getSensorIRCode,
    .setIRCtrlOutput = setIRCtrlOutput,
    .getSensorBatteryAD = getSensorBatteryAD,
    .getSensorRefVoltage = getSensorRefVoltage,
    .getSensorPotVoltage = getSensorPotVoltage,
    .getSensorPot = getSensorPot,
    .getMotorCurrent = getMotorCurrent,
    .getEncoderDirection = getEncoderDirection,
    .getEncoderPulse = getEncoderPulse,
    .getEncoderSpeed = getEncoderSpeed,
    .getCustomAD = getCustomAD,
    .getCustomDIn = getCustomDIn,
    .setCustomDOut = setCustomDOut,
    .setMotorPolarity = setMotorPolarity,
    .enableDCMotor = enableDCMotor,
    .disableDCMotor = disableDCMotor,
    .resumeDCMotor = resumeDCMotor,
    .suspendDCMotor = suspendDCMotor,
    .setDCMotorPositionPID = setDCMotorPositionPID,
    .setDCMotorVelocityPID = setDCMotorVelocityPID,
    .setDCMotorSensorFilter = setDCMotorSensorFilter,
    .setDCMotorSensorUsage = setDCMotorSensorUsage,
    .setDCMotorCtrlMode = setDCMotorCtrlMode,
    .setDCMotorPositionWithTime = setDCMotorPositionWithTime,
    .setDCMotorPosition = setDCMotorPosition,
    .setDCMotorPulseWithTime = setDCMotorPulseWithTime,
    .setDCMotorPulse = setDCMotorPulse,
    .setAllDCMotorPositionsWithTime = setAllDCMotorPositionsWithTime,
    .setAllDCMotorPositions = setAllDCMotorPositions,
    .setAllDCMotorVelocitiesWithTime = setAllDCMotorVelocitiesWithTime,
    .setAllDCMotorVelocities = setAllDCMotorVelocities,
    .setAllDCMotorPulsesWithTime = setAllDCMotorPulsesWithTime,
    .setAllDCMotorPulses = setAllDCMotorPulses,
    .setBothDCMotorPositionsWithTime = setBothDCMotorPositionsWithTime,
    .setBothDCMotorPositions = setBothDCMotorPositions,
    .setBothDCMotorVelocitiesWithTime = setBothDCMotorVelocitiesWithTime,
    .setBothDCMotorVelocities = setBothDCMotorVelocities,
    .setBothDCMotorPulsesWithTime = setBothDCMotorPulsesWithTime,
    .setBothDCMotorPulses = setBothDCMotorPulses,
    .enableServo = enableServo,
    .disableServo = disableServo,
    .setServoPulseWithTime = setServoPulseWithTime,
    .setServoPulse = setServoPulse,
    .setAllServoPulsesWithTime = setAllServoPulsesWithTime,
    .setAllServoPulses = setAllServoPulses,
    .setBothServoPulsesWithTime = setBothServoPulsesWithTime,
    .setBothServoPulses = setBothServoPulses,
    .setLCDDisplayPMS = setLCDDisplayPMS
};

#endif
