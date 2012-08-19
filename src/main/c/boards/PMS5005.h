#pragma once
#ifndef PMS5005_H
#define PMS5005_H

#include "../Buffer.h"

/* or typedef struct PMS5005 */
typedef struct PMS5005
{
    const int HEADER_LENGTH = 7;
    const int PAYLOAD_OFFSET = 6;
    const int DID_OFFSET = 4;

    const int MOTOR_SENSOR_REQUEST_SIZE = 10;
    const int STANDARD_SENSOR_REQUEST_SIZE = 10;
    const int CUSTOM_SENSOR_REQUEST_SIZE = 10;
    const int ALL_SENSOR_REQUEST_SIZE = 10;
    const int ENABLE_ALL_SENSOR_SENDING_SIZE = 9;
    const int ENABLE_STANDARD_SENSOR_SENDING_SIZE = 9;
    const int ENABLE_ALL_SENSOR_SENDING_SIZE = 9;
    const int DISABLE_MOTOR_SENSOR_SENDING_SIZE = 10;
    const int DISABLE_STANDARD_SENSOR_SENDING_SIZE = 10;
    const int DISABLE_ALL_SENSOR_SENDING_SIZE = 10;
    const int SET_MOTOR_POLARITY_SIZE = 12;
    const int ENABLE_DC_MOTOR_SIZE = 11;
    const int DISABLE_DC_MOTOR_SIZE = 11;
    const int RESUME_DC_MOTOR_SIZE = 11;
    const int SUSPEND_DC_MOTOR_SIZE = 11;
    const int SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = 20;
    const int SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = 20;
    const int SET_DC_MOTOR_SENSOR_FILTER_SIZE = 12;
    const int SET_DC_MOTOR_CTRL_MODE_SIZE = 12;
    const int DC_MOTOR_POSITION_TIME_CTRL_SIZE = 14;
    const int DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = 12;
    const int DC_MOTOR_PWM_TIME_CTRL_SIZE = 14;
    const int DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = 12;
    const int DC_MOTOR_POSITION_TIME_CTRL_ALL_SIZE = 23;
    const int DC_MOTOR_POSITION_NON_TIME_CTRL_ALL_SIZE = 21;
    const int DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = 23;
    const int DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = 21;
    const int DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = 23;
    const int DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = 21;
    const int ENABLE_SERVO_SIZE = 11;
    const int DISABLE_SERVO_SIZE = 11;
    const int SERVO_TIME_CTRL_SIZE = 15;
    const int SERVO_NON_TIME_CTRL_SIZE = 13;
    const int SERVO_TIME_CTRL_ALL_SIZE = 23;
    const int SERVO_NON_TIME_CTRL_ALL_SIZE = 21;

/* Start transmission, End transmission */
    const byte STX0 = 94;
    const byte STX1 = 2;
    const byte ETX0 = 94;
    const byte ETX1 = 13;

/* Data ID (DID) descriptor listing */
    const byte POSITION_CTRL_DID = 3;
    const byte ALL_POSITION_CTRL_DID = 4;
    const byte PWM_CTRL_DID = 5;
    const byte ALL_PWM_CTRL_DID = 6;
    const byte PARAM_SET_DID = 7;
	
/* SubCommands under PARAM_SET */
    const byte DC_POSITION_PID_DID = 7;     // positon PID Control
    const byte DC_VELOCITY_PID_DID = 8;     // velocity PID Control
    const byte DC_SENSOR_USAGE_DID = 13;
    const byte DC_CTRL_MODE_DID = 14;

    const byte POWER_CTRL_DID = 22;
    const byte LCD_CTRL_DID = 23;
    const byte VELOCITY_CTRL_DID = 26;
    const byte ALL_VELOCITY_CTRL_DID = 27;
    const byte SERVO_CTRL_DID = 28;
    const byte ALL_SERVO_CTRL_DID = 29;
    const byte TOGGLE_DC_MOTORS_DID = 30;
    const byte CONSTELLATION_CTRL_DID = 80;
    const byte GET_MOTOR_SENSOR_DATA_DID = 123;
    const byte GET_CUSTOM_SENSOR_DATA_DID = 124;
    const byte GET_STANDARD_SENSOR_DATA_DID = 125;
    const byte GET_ALL_SENSOR_DATA_DID = 127;
/* to use as ubyte: (byte)(SETUP_COM & 0xff) */
    const short SETUP_COM = 255;
/* End Data ID (DID) descriptor listing */

    const byte PWM_CTRL_MODE = 0;
    const byte POSITION_CTRL_MODE = 1;
    const byte VELOCITY_CTRL_MODE = 2;
    
    const byte KpId = 1; // progressive id
    const byte KdId = 2; // derivative id
    const byte KiId = 3; // integral id
    
    const short NON_CTRL_PACKET = 0xffff; // no ctrl command
    const short NO_CTRL = 0x8000;
	
/* Sensor Data Offsets */
    const int ULTRASONIC_OFFSET = 0 + HEADER_LENGTH;
    const int ENCODER_PULSE_OFFSET = 24 + HEADER_LENGTH; 
    const int ENCODER_SPEED_OFFSET = 32 + HEADER_LENGTH;
    const int STANDARD_IR_RANGE_OFFSET = 24 + HEADER_LENGTH;
    const int CUSTOM_IR_RANGE_OFFSET = 4 + HEADER_LENGTH; // CustomAD3
    const int HUMAN_ALARM_OFFSET = 6 + HEADER_LENGTH;
    const int HUMAN_MOTION_OFFSET = 8 + HEADER_LENGTH;
    const int TILTING_X_OFFSET = 14 + HEADER_LENGTH;
    const int TILTING_Y_OFFSET = 16 + HEADER_LENGTH;
    const int ENCODER_DIRECTION_OFFSET = 32 + HEADER_LENGTH;
    const int MOTOR_SPEED_OFFSET = 26 + HEADER_LENGTH;
    const int CUSTOM_AD_OFFSET = 0 + HEADER_LENGTH;
    const int TEMPERATURE_AD_OFFSET = 22 + HEADER_LENGTH;
    const int OVERHEAT_SENSOR_OFFSET = 18 + HEADER_LENGTH;
    const int INFRARED_COMMAND_OFFSET = 26 + HEADER_LENGTH;
    const int BATTERY_SENSOR_OFFSET = 30 + HEADER_LENGTH;
    const int REFERENCE_VOLTAGE_OFFSET = 36 + HEADER_LENGTH;
    const int POTENTIOMETER_POWER_OFFSET = 38 + HEADER_LENGTH;
    const int POTENTIOMETER_SENSOR_OFFSET = 0 + HEADER_LENGTH;
    const int MOTOR_CURRENT_SENSOR_OFFSET = 12 + HEADER_LENGTH;

    /* methods */
    byte (*crc)();
    Buffer* (*motorSensorRequest)(short packetNumber);
    Buffer* (*standardSensorRequest)(short packetNumber);
    Buffer* (*customSensorRequest)(short packetNumber);
    Buffer* (*allSensorRequest)(short packetNumber);
    Buffer* (*enableMotorSensorSending)();
    Buffer* (*enableStandardSensorSending)();
    Buffer* (*enableCustomSensorSending)();
    Buffer* (*enableAllSensorSending)();
    Buffer* (*disableMotorSensorSending)();
    Buffer* (*disableStandardSensorSending)();
    Buffer* (*disableCustomSensorSending)();
    Buffer* (*disableAllSensorSending)();
    Buffer* (*setMotorSensorPeriod)(short);
    Buffer* (*setStandardSensorPeriod)(short);
    Buffer* (*setCustomSensorPeriod)(short);
    Buffer* (*setAllSensorPeriod)(short);
    short (*getSensorSonar)(short, const int[]);
    short (*getSensorIrRange)(short, const int[], const int[]);
    short (*getSensorHumanAlarm)(short, const int[]);
    short (*getSensorHumanMotion)(short, const int[]);
    short (*getSensorTiltingX)(const int[]);
    short (*getSensorTiltingY)(const int[]);
    short (*getSensorOverHeat)(short, const int[]);
    short (*getSensorTemperature)(const int[]);
    short (*getSensorIrCode)(const int[]);
    Buffer* (*setIrCtrlOutput)(short, short);
    short (*getSensorBatteryAd)(short, const int[]);
    short (*getSensorRefVoltage)(const int[]);
    short (*getSensorPotVoltage)(const int[]);
    short (*getSensorPot)(short, const int[]);
    short (*getMotorCurrent)(short, const int[]);
    short (*getEncoderDirection)(short, const int[]);
    short (*getEncoderPulse)(short, const int[]);
    short (*getEncoderSpeed)(short, const int[]);
    short (*getCustomAd)(short, const int[]);
    short (*getCustomDIn)(byte, const int[]);
    Buffer* (*getCustomDOut)(byte);
    Buffer* (*setMotorPolarity)(byte, byte);
    Buffer* (*enableDcMotor)(byte);
    Buffer* (*disableDcMotor)(byte);
    Buffer* (*resumeDcMotor)(byte);
    Buffer* (*suspendDcMotor)(byte);
    Buffer* (*setDcMotorPositionCtrlPid)(byte, short, short, short);
    Buffer* (*setDcMotorVelocityCtrlPid)(byte, short, short, short);
    Buffer* (*setDcMotorSensorFilter)(byte, short);
    Buffer* (*setDcMotorSensorUsage)(byte, byte);
    Buffer* (*setDcMotorCtrlMode)(byte, byte);
    Buffer* (*dcMotorPositionTimeCtrl)(byte, short, short);
    Buffer* (*dcMotorPositionNonTimeCtrl)(byte, short);
    Buffer* (*dcMotorPwmTimeCtrl)(byte, short, short);
    Buffer* (*dcMotorPwmNonTimeCtrl)(byte, short);
    Buffer* (*dcMotorPositionTimeCtrlAll)(short, short, short, short, short, short, short);
    Buffer* (*dcMotorPositionNonTimeCtrlAll)(short, short, short, short, short, short);
    Buffer* (*dcMotorPositionTimeCtrlBoth)(short, short, short);
    Buffer* (*dcMotorPositionNonTimeCtrlBoth)(short, short);
    Buffer* (*dcMotorVelocityTimeCtrlAll)(short, short, short, short, short, short, short);
    Buffer* (*dcMotorVelocityNonTimeCtrlAll)(short, short, short, short, short, short);
    Buffer* (*dcMotorVelocityTimeCtrlBoth)(short, short, short);
    Buffer* (*dcMotorVelocityNonTimeCtrlBoth)(short, short);
    Buffer* (*dcMotorPwmTimeCtrlAll)(short, short, short, short, short, short, short);
    Buffer* (*dcMotorPwmNonTimeCtrlAll)(short, short, short, short, short, short);
    Buffer* (*dcMotorPwmTimeCtrlBoth)(short, short, short);
    Buffer* (*dcMotorPwmNonTimeCtrlBoth)(short, short);
    Buffer* (*enableServo)(byte);
    Buffer* (*disableServo)(byte);
    Buffer* (*servoTimeCtrl)(byte, short, short);
    Buffer* (*servoNonTimeCtrl)(byte, short);
    Buffer* (*servoTimeCtrlAll)(short, short, short, short, short, short, short);
    Buffer* (*servoNonTimeCtrlAll)(short, short, short, short, short, short);
    Buffer* (*servoTimeCtrlBoth)(short, short, short);
    Buffer* (*servoNonTimeCtrlBoth)(short, short);
    Buffer* (*lcdDisplayPms)(char*);
} PMS5005;

#endif
