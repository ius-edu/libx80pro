#pragma once
#ifndef PMS5005_H
#define PMS5005_H

#include "../Buffer.h"

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
    
    unsigned char KPID; /* progressive id */
    unsigned char KDID; /* derivative id */
    unsigned char KIID; /* integral id */
    
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
    unsigned char (*crc)();
    Buffer (*motorSensorRequest)(struct PMS5005, short);
    Buffer (*standardSensorRequest)(struct PMS5005, short);
    Buffer (*customSensorRequest)(struct PMS5005, short);
    Buffer (*allSensorRequest)(struct PMS5005, short);
    Buffer (*enableMotorSensorSending)(struct PMS5005);
    Buffer (*enableStandardSensorSending)(struct PMS5005);
    Buffer (*enableCustomSensorSending)(struct PMS5005);
    Buffer (*enableAllSensorSending)(struct PMS5005);
    Buffer (*disableMotorSensorSending)(struct PMS5005);
    Buffer (*disableStandardSensorSending)(struct PMS5005);
    Buffer (*disableCustomSensorSending)(struct PMS5005);
    Buffer (*disableAllSensorSending)(struct PMS5005);
    Buffer (*setMotorSensorPeriod)(struct PMS5005, short);
    Buffer (*setStandardSensorPeriod)(struct PMS5005, short);
    Buffer (*setCustomSensorPeriod)(struct PMS5005, short);
    Buffer (*setAllSensorPeriod)(struct PMS5005, short);
    short  (*getSensorSonar)(struct PMS5005, short, const int[]);
    short  (*getSensorIrRange)(struct PMS5005, short, const int[], const int[]);
    short  (*getSensorHumanAlarm)(struct PMS5005, short, const int[]);
    short  (*getSensorHumanMotion)(struct PMS5005, short, const int[]);
    short  (*getSensorTiltingX)(struct PMS5005, const int[]);
    short  (*getSensorTiltingY)(struct PMS5005, const int[]);
    short  (*getSensorOverheat)(struct PMS5005, short, const int[]);
    short  (*getSensorTemperature)(struct PMS5005, const int[]);
    short  (*getSensorIrCode)(struct PMS5005, short, const int[]);
    Buffer (*setIrCtrlOutput)(struct PMS5005, short, short);
    short  (*getSensorBatteryAd)(struct PMS5005, short, const int[]);
    short  (*getSensorRefVoltage)(struct PMS5005, const int[]);
    short  (*getSensorPotVoltage)(struct PMS5005, const int[]);
    short  (*getSensorPot)(struct PMS5005, short, const int[]);
    short  (*getMotorCurrent)(struct PMS5005, short, const int[]);
    short  (*getEncoderDirection)(struct PMS5005, short, const int[]);
    short  (*getEncoderPulse)(struct PMS5005, short, const int[]);
    short  (*getEncoderSpeed)(struct PMS5005, short, const int[]);
    short  (*getCustomAd)(struct PMS5005, short, const int[]);
    short  (*getCustomDIn)(struct PMS5005, unsigned char, const int[]);
    Buffer (*setCustomDOut)(struct PMS5005, unsigned char);
    Buffer (*setMotorPolarity)(struct PMS5005, unsigned char, unsigned char);
    Buffer (*enableDcMotor)(struct PMS5005, unsigned char);
    Buffer (*disableDcMotor)(struct PMS5005, unsigned char);
    Buffer (*resumeDcMotor)(struct PMS5005, unsigned char);
    Buffer (*suspendDcMotor)(struct PMS5005, unsigned char);
    Buffer (*setDcMotorPositionCtrlPid)(struct PMS5005, unsigned char, short, short, short);
    Buffer (*setDcMotorVelocityCtrlPid)(struct PMS5005, unsigned char, short, short, short);
    Buffer (*setDcMotorSensorFilter)(struct PMS5005, unsigned char, short);
    Buffer (*setDcMotorSensorUsage)(struct PMS5005, unsigned char, unsigned char);
    Buffer (*setDcMotorCtrlMode)(struct PMS5005, unsigned char, unsigned char);
    Buffer (*dcMotorPositionTimeCtrl)(struct PMS5005, unsigned char, short, short);
    Buffer (*dcMotorPositionNonTimeCtrl)(struct PMS5005, unsigned char, short);
    Buffer (*dcMotorPwmTimeCtrl)(struct PMS5005, unsigned char, short, short);
    Buffer (*dcMotorPwmNonTimeCtrl)(struct PMS5005, unsigned char, short);
    Buffer (*dcMotorPositionTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short, short);
    Buffer (*dcMotorPositionNonTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short);
    Buffer (*dcMotorPositionTimeCtrlBoth)(struct PMS5005, short, short, short);
    Buffer (*dcMotorPositionNonTimeCtrlBoth)(struct PMS5005, short, short);
    Buffer (*dcMotorVelocityTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short, short);
    Buffer (*dcMotorVelocityNonTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short);
    Buffer (*dcMotorVelocityTimeCtrlBoth)(struct PMS5005, short, short, short);
    Buffer (*dcMotorVelocityNonTimeCtrlBoth)(struct PMS5005, short, short);
    Buffer (*dcMotorPwmTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short, short);
    Buffer (*dcMotorPwmNonTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short);
    Buffer (*dcMotorPwmTimeCtrlBoth)(struct PMS5005, short, short, short);
    Buffer (*dcMotorPwmNonTimeCtrlBoth)(struct PMS5005, short, short);
    Buffer (*enableServo)(struct PMS5005, unsigned char);
    Buffer (*disableServo)(struct PMS5005, unsigned char);
    Buffer (*servoTimeCtrl)(struct PMS5005, unsigned char, short, short);
    Buffer (*servoNonTimeCtrl)(struct PMS5005, unsigned char, short);
    Buffer (*servoTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short, short);
    Buffer (*servoNonTimeCtrlAll)(struct PMS5005, short, short, short, short, short, short);
    Buffer (*servoTimeCtrlBoth)(struct PMS5005, short, short, short);
    Buffer (*servoNonTimeCtrlBoth)(struct PMS5005, short, short);
    Buffer (*lcdDisplayPms)(struct PMS5005, char*);
} PMS5005;

#endif
