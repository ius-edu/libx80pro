#pragma once
#ifndef PMS5005_H
#define PMS5005_H

#include "../buffer/Buffer.h"

short PMS5005_HEADER_LENGTH = 7;
short PMS5005_PAYLOAD_OFFSET = 6;
short PMS5005_DID_OFFSET = 4;
short PMS5005_MOTOR_SENSOR_REQUEST_SIZE = 10;
short PMS5005_STANDARD_SENSOR_REQUEST_SIZE = 10;
short PMS5005_CUSTOM_SENSOR_REQUEST_SIZE = 10;
short PMS5005_ALL_SENSOR_REQUEST_SIZE = 10;
short PMS5005_ENABLE_ALL_SENSOR_SENDING_SIZE = 9;
short PMS5005_ENABLE_STANDARD_SENSOR_SENDING_SIZE = 9;
short PMS5005_ENABLE_CUSTOM_SENSOR_SENDING_SIZE = 9;
short PMS5005_DISABLE_MOTOR_SENSOR_SENDING_SIZE = 10;
short PMS5005_DISABLE_STANDARD_SENSOR_SENDING_SIZE = 10;
short PMS5005_DISABLE_ALL_SENSOR_SENDING_SIZE = 10;
short PMS5005_SET_MOTOR_POLARITY_SIZE = 12;
short PMS5005_ENABLE_DC_MOTOR_SIZE = 11;
short PMS5005_DISABLE_DC_MOTOR_SIZE = 11;
short PMS5005_RESUME_DC_MOTOR_SIZE = 11;
short PMS5005_SUSPEND_DC_MOTOR_SIZE = 11;
short PMS5005_SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = 20;
short PMS5005_SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = 20;
short PMS5005_SET_DC_MOTOR_SENSOR_FILTER_SIZE = 12;
short PMS5005_SET_DC_MOTOR_CTRL_MODE_SIZE = 12;
short PMS5005_DC_MOTOR_POSITION_TIME_CTRL_SIZE = 14;
short PMS5005_DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 12;
short PMS5005_DC_MOTOR_PWM_TIME_CTRL_SIZE = 14;
short PMS5005_DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = 12;
short PMS5005_DC_MOTOR_POSITION_TIME_CTRL_ALL_SIZE = 23;
short PMS5005_DC_MOTOR_POSITION_NON_TIME_CTRL_ALL_SIZE = 21;
short PMS5005_DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = 23;
short PMS5005_DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = 21;
short PMS5005_DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = 23;
short PMS5005_DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = 21;
short PMS5005_ENABLE_SERVO_SIZE = 11;
short PMS5005_DISABLE_SERVO_SIZE = 11;
short PMS5005_SERVO_TIME_CTRL_SIZE = 15;
short PMS5005_SERVO_NON_TIME_CTRL_SIZE = 13;
short PMS5005_SERVO_TIME_CTRL_ALL_SIZE = 23;
short PMS5005_SERVO_NON_TIME_CTRL_ALL_SIZE = 21;
short PMS5005_STX0 = 94;
short PMS5005_STX1 = 2;
short PMS5005_ETX0 = 94;
short PMS5005_ETX1 = 13;
short PMS5005_POSITION_CTRL_DID = 3;
short PMS5005_ALL_POSITION_CTRL_DID = 4;
short PMS5005_PWM_CTRL_DID = 5;
short PMS5005_ALL_PWM_CTRL_DID = 6;
short PMS5005_PARAM_SET_DID = 7;
short PMS5005_DC_POSITION_PID_DID = 7;
short PMS5005_DC_VELOCITY_PID_DID = 8;
short PMS5005_DC_SENSOR_USAGE_DID = 13;
short PMS5005_DC_CTRL_MODE_DID = 14;
short PMS5005_POWER_CTRL_DID = 22;
short PMS5005_LCD_CTRL_DID = 23;
short PMS5005_VELOCITY_CTRL_DID = 26;
short PMS5005_ALL_VELOCITY_CTRL_DID = 27;
short PMS5005_SERVO_CTRL_DID = 28;
short PMS5005_ALL_SERVO_CTRL_DID = 29;
short PMS5005_TOGGLE_DC_MOTORS_DID = 30;
short PMS5005_CONSTELLATION_CTRL_DID = 80;
short PMS5005_GET_MOTOR_SENSOR_DATA_DID = 123;
short PMS5005_GET_CUSTOM_SENSOR_DATA_DID = 124;
short PMS5005_GET_STANDARD_SENSOR_DATA_DID = 125;
short PMS5005_GET_ALL_SENSOR_DATA_DID = 127;
short PMS5005_SETUP_COM = 255;
short PMS5005_PWM_CTRL_MODE = 0;
short PMS5005_POSITION_CTRL_MODE = 1;
short PMS5005_VELOCITY_CTRL_MODE = 2;
short PMS5005_KP_ID = 1;
short PMS5005_KD_ID = 2;
short PMS5005_KI_ID = 3;
short PMS5005_NON_CTRL_PACKET = (short)0xffff;
short PMS5005_NO_CTRL = (short)0x8000;
short PMS5005_ULTRASONIC_OFFSET = 7; /* 0 + PMS5005_HEADER_LENGTH */
short PMS5005_ENCODER_PULSE_OFFSET = 31; /* 24 + PMS5005_HEADER_LENGTH */
short PMS5005_ENCODER_SPEED_OFFSET = 39; /* 32 + PMS5005_HEADER_LENGTH */
short PMS5005_STANDARD_IR_RANGE_OFFSET = 31; /* 24 + PMS5005_HEADER_LENGTH */
short PMS5005_CUSTOM_IR_RANGE_OFFSET = 11; /* 4 + PMS5005_HEADER_LENGTH */
short PMS5005_HUMAN_ALARM_OFFSET = 13; /* 6 + PMS5005_HEADER_LENGTH */
short PMS5005_HUMAN_MOTION_OFFSET = 15; /* 8 + PMS5005_HEADER_LENGTH */
short PMS5005_TILTING_X_OFFSET = 21; /* 14 + PMS5005_HEADER_LENGTH */
short PMS5005_TILTING_Y_OFFSET = 23; /* 16 + PMS5005_HEADER_LENGTH */
short PMS5005_ENCODER_DIRECTION_OFFSET = 39; /* 32 + PMS5005_HEADER_LENGTH */
short PMS5005_MOTOR_SPEED_OFFSET = 33; /* 26 + PMS5005_HEADER_LENGTH */
short PMS5005_CUSTOM_AD_OFFSET = 7; /* 0 + PMS5005_HEADER_LENGTH */
short PMS5005_TEMPERATURE_AD_OFFSET = 29; /* 22 + PMS5005_HEADER_LENGTH */
short PMS5005_OVERHEAT_SENSOR_OFFSET = 25; /* 18 + PMS5005_HEADER_LENGTH */
short PMS5005_IR_COMMAND_OFFSET = 33; /* 26 + PMS5005_HEADER_LENGTH */
short PMS5005_BATTERY_SENSOR_OFFSET = 37; /* 30 + PMS5005_HEADER_LENGTH */
short PMS5005_REFERENCE_VOLTAGE_OFFSET = 43; /* 36 + PMS5005_HEADER_LENGTH */
short PMS5005_POTENTIOMETER_POWER_OFFSET = 45; /* 38 + PMS5005_HEADER_LENGTH */
short PMS5005_POTENTIOMETER_SENSOR_OFFSET = 7; /* 0 + PMS5005_HEADER_LENGTH */
short PMS5005_MOTOR_CURRENT_SENSOR_OFFSET = 19; /* 12 + PMS5005_HEADER_LENGTH */

unsigned char PMS5005_calcCRC(Buffer *buf);
const Buffer * PMS5005_motorSensorRequest(Buffer *buf, short packetNumber);
const Buffer * PMS5005_standardSensorRequest(Buffer *buf, short packetNumber);
const Buffer * PMS5005_customSensorRequest(Buffer *buf, short packetNumber);
const Buffer * PMS5005_allSensorRequest(Buffer *buf, short packetNumber);
const Buffer * PMS5005_enableMotorSensorSending(Buffer *buf);
const Buffer * PMS5005_enableStandardSensorSending(Buffer *buf);
const Buffer * PMS5005_enableCustomSensorSending(Buffer *buf);
const Buffer * PMS5005_enableAllSensorSending(Buffer *buf);
const Buffer * PMS5005_disableMotorSensorSending(Buffer *buf);
const Buffer * PMS5005_disableStandardSensorSending(Buffer *buf);
const Buffer * PMS5005_disableCustomSensorSending(Buffer *buf);
const Buffer * PMS5005_disableAllSensorSending(Buffer *buf);
const Buffer * PMS5005_setMotorSensorPeriod(Buffer *buf, short timePeriod);
const Buffer * PMS5005_setStandardSensorPeriod(Buffer *buf, short timePeriod);
const Buffer * PMS5005_setCustomSensorPeriod(Buffer *buf, short timePeriod);
const Buffer * PMS5005_setAllSensorPeriod(Buffer *buf, short timePeriod);
const Buffer * PMS5005_setIRCtrlOutput(Buffer *buf, short loWord, short hiWord);
const Buffer * PMS5005_setCustomDOut(Buffer *buf, unsigned char ival);
const Buffer * PMS5005_setMotorPolarity(Buffer *buf, unsigned char channel, unsigned char polarity);
const Buffer * PMS5005_enableDCMotor(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_disableDCMotor(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_resumeDCMotor(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_suspendDCMotor(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_setDCMotorPositionPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
const Buffer * PMS5005_setDCMotorVelocityPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100);
const Buffer * PMS5005_setDCMotorSensorFilter(Buffer *buf, unsigned char channel, short filterMethod);
const Buffer * PMS5005_setDCMotorSensorUsage(Buffer *buf, unsigned char channel, unsigned char sensorType);
const Buffer * PMS5005_setDCMotorCtrlMode(Buffer *buf, unsigned char channel, unsigned char controlMode);
const Buffer * PMS5005_setDCMotorPositionWithTime(Buffer *buf, unsigned char channel, short pos, short timePeriod);
const Buffer * PMS5005_setDCMotorPosition(Buffer *buf, unsigned char channel, short pos);
const Buffer * PMS5005_setDCMotorPulseWithTime(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod);
const Buffer * PMS5005_setDCMotorPulse(Buffer *buf, unsigned char channel, short pulseWidth);
const Buffer * PMS5005_setAllDCMotorPositionsWithTime(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod);
const Buffer * PMS5005_setAllDCMotorPositions(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5);
const Buffer * PMS5005_setBothDCMotorPositionsWithTime(Buffer *buf, short posLeft, short posRight, short timePeriod);
const Buffer * PMS5005_setBothDCMotorPositions(Buffer *buf, short posLeft, short posRight);
const Buffer * PMS5005_setAllDCMotorVelocitiesWithTime(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod);
const Buffer * PMS5005_setAllDCMotorVelocities(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5);
const Buffer * PMS5005_setBothDCMotorVelocitiesWithTime(Buffer *buf, short vLeft, short vRight, short timePeriod);
const Buffer * PMS5005_setBothDCMotorVelocities(Buffer *buf, short vLeft, short vRight);
const Buffer * PMS5005_setAllDCMotorPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
const Buffer * PMS5005_setAllDCMotorPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
const Buffer * PMS5005_setBothDCMotorPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod);
const Buffer * PMS5005_setBothDCMotorPulses(Buffer *buf, short pLeft, short pRight);
const Buffer * PMS5005_enableServo(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_disableServo(Buffer *buf, unsigned char channel);
const Buffer * PMS5005_setServoPulseWithTime(Buffer *buf, unsigned char channel, short pulse, short timePeriod);
const Buffer * PMS5005_setServoPulse(Buffer *buf, unsigned char channel, short pulse);
const Buffer * PMS5005_setAllServoPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod);
const Buffer * PMS5005_setAllServoPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5);
const Buffer * PMS5005_setBothServoPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod);
const Buffer * PMS5005_setBothServoPulses(Buffer *buf, short pLeft, short pRight);
const Buffer * PMS5005_setLCDDisplayPMS(Buffer *buf, char *bmpFileName);

#endif
