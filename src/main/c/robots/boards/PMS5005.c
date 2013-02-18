#include <stdlib.h>
#include <assert.h>

#include "PMS5005.h"
#include "../buffer/Buffer.h"

/*
 * C implmentation written by Jesse Riddle and Colton Jenkins, with
 * significant documentation taken from the DrRobot WiRobot SDK Application
 * Programming Interface (API) Reference Manual - (For MS Windows)
 * Version: 1.0.8 Feb. 2004 by DrRobot, Inc. and some parts from the DrRobot
 * Java (yes, Java) motion control demo program.
 */

/*
 * The structure for a packet to the PMS5005 is the following: STX0 = 94
 * (always) STX1 = 2 (always) RID = 1 (generally) Reserved = 0 (generally)
 * DID (Data ID) = <DID> (your choice of DID) LENGTH = <len> (Length from
 * next element to crc unsigned char) DATA = <data> (may be more than 1 unsigned char in
 * length) CHECKSUM = <cksum> (use crc() method to calculate on packet) ETX0 =
 * 94 (always) ETX1 = 13 (always)
 */

static int test()
{
	return 73;
}

/**
 * Calculates a valid crc value to be used in order to check the integrity
 * of the contents of a request packet.
 *
 * @param packet is the packetfer from which the crc value will be calculated.
 *
 * @return The crc value calculated from the given packetfer.
 */
static unsigned char calcCRC(Buffer *buf)
{
    unsigned char shift_reg, sr_lsb, data_bit, v;
    int i, j, z;
    unsigned char fb_bit;
    shift_reg = 0;               // initialize the shift register
    /* z = bufSize - 3; // Don't include crc and ETX (z=length-3) */
    z = buf->length;
    for (i = 0; i < z; ++i)// Don't include STX (i=2)
    {
        /* v = packet[i + 2] & 0x0000ffff; */
        v = buf->data[i] & 0x0000ffff;
        // for each bit
        for (j = 0; j < 8; ++j)
        {
            // isolate least sign bit
            data_bit = v & 0x01;
            sr_lsb = shift_reg & 0x01;
            // calculate the feed back bit
            fb_bit = (data_bit ^ sr_lsb) & 0x01;
            shift_reg = shift_reg >> 1;
            if (fb_bit == 1)
            {
                shift_reg = shift_reg ^ 0x8C;
            }
            v = v >> 1;
        }
    }
    
    return shift_reg;
}

/**
 * Sends a request command to the Sensing and Motion Controller (PMS5005)
 * in order to get the sensor data related to motor control.
 *
 * @param nPacket describes how many times sampled data should be sent
 *     nPacket == 0 -- Stop sending the sensor data packets.
 *     nPacket == -1 -- Send sensor data packets continuously
 *                          until being asked to stop (default).
 *     nPacket > 0 -- Send n = nPacket packets of sensor data
 *                         and then stop sending.
 *
 * Please Note:
 * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or
 * 50ms (default).
 *
 * See Also: setMotorSensorPeriod
 */
static Buffer * motorSensorRequest(Buffer *buf, short packetNumber)
{
    assert (buf != NULL);

	buf->length = 10;
    
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_MOTOR_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = packetNumber;
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;

	return buf;
}

/**
 * Sends a request command to the Sensing and Motion Controller (PMS5005)
 * in order to get all the standard sensor data.
 *
 * @param packetNumber describes how many times sampled data should be
 * sent.
 *     packetNumber == 0 -- Stop sending the sensor data packets.
 *     packetNumber == -1 -- Send sensor data packets continuously
 *                          until being asked to stop.
 *     packetNumber > 0 -- Send n = packetNumber packets of sensor data
 *                         and then stop sending.
 *
 * Please Note:
 * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or
 * 50ms (default).
 *
 * See Also: setStandardSensorPeriod
 */
static Buffer * standardSensorRequest(Buffer *buf, short packetNumber)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_MOTOR_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = packetNumber;
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
    
    return buf;
}

/**
 * Sends a request command to the Sensing and Motion Controller (PMS5005)
 * in order to get all custom sensor data.
 *
 * @param packetNumber describes how many times sampled data should be
 * sent.
 *     packetNumber == 0 -- Stop sending the sensor data packets.
 *     packetNumber == 1 -- Send sensor data packets continuously
 *                          until being asked to stop (default).
 *     packetNumber > 0 -- Send n = packetNumber packets of sensor data
 *                         and then stop sending.
 *
 * Please Note:
 * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or
 * 50ms (default).
 *
 * See Also: setCustomSensorPeriod
 */
static Buffer * customSensorRequest(Buffer *buf, short packetNumber)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_CUSTOM_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = packetNumber;
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
    
    return buf;
}

/**
 * Sends a request command to the Sensing and Motion Controller (PMS5005)
 * in order to get all sensor data.
 *
 * @param packetNumber describes how many times sampled data should be
 * sent.
 *     packetNumber == 0 -- Stop sending the sensor data packets.
 *     packetNumber == 1 -- Send sensor data packets continuously
 *                          until being asked to stop.
 *     packetNumber > 0 -- Send n = packetNumber packets of sensor data
 *                         and then stop sending.
 *
 * Please note:
 * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or
 * 50ms (default).
 *
 * See Also: setAllSensorPeriod
 */
static Buffer * allSensorRequest(Buffer *buf, short packetNumber)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_ALL_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = packetNumber;
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
    
    return buf;
}

/**
 * Enables batch updating of motor-related sensor packets.
 *
 * Please note:
 * 1) The latest request setting of the packet number and the update rates
 *    are used.
 * 2) By default, "all sensor data sending" is enabled.
 *
 * @see motorSensorRequest
 */
static Buffer * enableMotorSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 9;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_MOTOR_SENSOR_DATA_DID;
	buf->data[5] = 0; // len
	buf->data[6] = calcCRC(buf);
	buf->data[7] = ETX0;
	buf->data[8] = ETX1;
    
    return buf;
}

/**
 * Enables batch updating of standard sensor packets.
 *
 * Please note:
 * 1) The latest request setting of the packet number and the update rates
 *   are used.
 * 2) By default, "all sensor data sending" is enabled.
 *
 * @see disableMotorSensorSending
 */
static Buffer * enableStandardSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 9;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_STANDARD_SENSOR_DATA_DID;
	buf->data[5] = 0; // len
	buf->data[6] = calcCRC(buf);
	buf->data[7] = ETX0;
	buf->data[8] = ETX1;
    
    return buf;
}

/**
 * Enables batch updating of custom sensor packets.
 *
 * Please note:
 * 1) The latest request setting of the packet number and the update rates
 *    are used.
 * 2) By default, "all sensor data sending" is enabled.
 *
 * @see disableStandardSensorSending
 */
static Buffer * enableCustomSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 9;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_CUSTOM_SENSOR_DATA_DID;
	buf->data[5] = 0; // len
	buf->data[6] = calcCRC(buf);
	buf->data[7] = ETX0;
	buf->data[8] = ETX1;
    
    return buf;
}

/**
 * Enables batch updating of all sensor packets.
 *
 * Please note:
 * 1) The latest request setting of the packet number and the update rates
 *    are used.
 * 2) By default, "all sensor data sending" is enabled.
 *
 * @see disableCustomSensorSending
 */
static Buffer * enableAllSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 9;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_ALL_SENSOR_DATA_DID;
	buf->data[5] = 0; // len
	buf->data[6] = calcCRC(buf);
	buf->data[7] = ETX0;
	buf->data[8] = ETX1;
    
    return buf;
}

/**
 * Disables batch updating of motor-related sensor packets.
 *
 * @see enableMotorSensorSending
 */
static Buffer * disableMotorSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_MOTOR_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = 0; //(0 & 0xff);
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;

    
    return buf;
}

/**
 * Disables batch updating of standard sensor packets.
 *
 * @see enableStandardSensorSending
 */
static Buffer * disableStandardSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_STANDARD_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = 0; //(0 & 0xff);
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
	
    return buf;
}

/**
 * Disables batch updating of custom sensor packets.
 *
 * @see enableCustomSensorSending
 */
static Buffer * disableCustomSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_CUSTOM_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = 0; //(0 & 0xff);
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
    
    return buf;
}

/**
 * Disables batch updating of all sensor packets.
 *
 * @see enableAllSensorSending
 */
static Buffer * disableAllSensorSending(Buffer *buf)
{
    assert (buf != NULL);

	buf->length = 10;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = GET_ALL_SENSOR_DATA_DID;
	buf->data[5] = 1; // len
	buf->data[6] = 0; //(0 & 0xff);
	buf->data[7] = calcCRC(buf);
	buf->data[8] = ETX0;
	buf->data[9] = ETX1;
    
    return buf;
}

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
/*
static Buffer * setMotorSensorPeriod(Buffer *buf, short timePeriod)
{
    // TODO stub
    return NULL;
}
*/

/**
 * Sets the refresh rate for batch updating by standard sensor packets.
 *
 * @param timePeriod The update period in milliseconds for batch sensing
 * packets to the PC central controller.
 *
 * Please note: The default timePeriod is 50ms.  The maximum refresh rate
 * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
 *
 * @see standardSensorRequest
 */
/*
static Buffer * setStandardSensorPeriod(Buffer *buf, short timePeriod)
{
    // TODO stub
    return NULL;
}
*/

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
/*
static Buffer * setCustomSensorPeriod(Buffer *buf, short timePeriod)
{
    // TODO stub
    return NULL;
}
*/

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
/*
static Buffer * setAllSensorPeriod(Buffer *buf, short timePeriod)
{
    // TODO stub
    return NULL;
}
*/

/**
 * Sends two 16-bit words of infrared communication output data to the
 * Sensing and Motion Controller (PMS5005).
 *
 * The PMS5005 will then send the data out through the infrared Remote
 * Controller Module (MIR5500).  In the case it is used as an infrared
 * remote control, the output data serves as the remote control command.
 *
 * @param lowWord First word
 * @param highWord Second word
 *
 * Please note:
 * 1) With infrared communication, the data format and interpretation can
 *    be defined by the user at the application level.
 * 2) The control command should be compatible with the device to which
 *    the command is sent.
 * 3) This API method is under development and will be available soon.
 */
static Buffer * setIRCtrlOutput(Buffer *buf, short loWord, short hiWord)
{
    // TODO Auto-generated method stub
    return NULL;
}

/**
 * Returns the current value of the power supply voltage for the channel
 * specified.
 *
 * Returns the current value of the relevant power supply voltage if the
 * battery voltage monitor is enabled (default).
 * If the battery voltage monitor is disabled, this method
 * returns the relevant custom A/D inputs, if the custom A/D input is
 * enabled -- which may be configured by the jumpers on the PMS5005 board.
 * Please refer to the PMS5005 Robot Sensing and Motion Controller User
 * Manual for detailed information on hardware settings.
 *
 * @param channel 0 -- battery of DSP circuits (or custom A/D channel #1)
 * channel 1 -- battery of DC motors (or custom A/D channel #2)
 * channel 2 -- battery for servo motors (or custom A/D channel #3)
 *
 * @return The raw value of the analog to digital converter indicating
 * the output voltage of the monitor.  The data range is between 0 and 4095
 *
 * Please note: When monitoring the voltage of the power supply, the
 * following equations can be used to calculate the real voltage values.
 * 1) Power supply voltage of DSP circuits = 9v*(ival/4095)
 * 2) Power supply voltage of DC motors = 24v*(ival/4095)
 * 3) Power supply voltage of servo motors = 9v*(ival/4095)
 */
/*
static short getSensorBatteryAD(const int *standardSensorData, short channel)
{
    return standardSensorData[2*channel + BATTERY_SENSOR_OFFSET + 1] << 8 | standardSensorData[2*channel + BATTERY_SENSOR_OFFSET];
}
*/

/**
 * Returns the current value of the reference voltage of the A/D converter
 * of the controller DSP.
 *
 * @return The raw value of the analog to digital converter indicating the
 * output voltage of the monitor.  The data range is between 0 and 4095.
 * The following equation can be used to calculate the actual voltage
 * values: Voltage = 6v*(ival/4095)
 */
/*
static short getSensorRefVoltage(const int *standardSensorData)
{
    return standardSensorData[REFERENCE_VOLTAGE_OFFSET + 1] << 8 | standardSensorData[REFERENCE_VOLTAGE_OFFSET];
}
*/

/**
 * Returns the current value of the reference voltage of the A/D converter
 * of the controller DSP.
 *
 * @return The raw value of the analog to digital converter indicating the
 * output voltage of the monitor.  The data range is between 0 and 4095.
 * The following equation can be used to calculate the actual voltage
 * values: Voltage = 6v*(ival/4095)
 */
/*
static short getSensorPotVoltage(const int *standardSensorData)
{
    return standardSensorData[POTENTIOMETER_POWER_OFFSET + 1] << 8 | standardSensorData[POTENTIOMETER_POWER_OFFSET];
}
*/

/**
 * Returns the current value of the specified potentiometer position sensor
 *
 * @param channel 0, 1, 2, 3, 4, or 5 for Potentiometer sensor #1 through 6
 *
 * @return The raw value given by the analog to digital converter
 * indicating the output voltage of the sensor.  The data range is between
 * 0 and 4095.  The angular position can be calculated as follows, with the
 * 180 degree position defind at the sensors' physical middle position.
 * Single sensor or dual sensor can be used for rotation measurement.
 *
 * Please note:
 * 1) Single sensor mode is mainly used for the control of a robot joshort
 *    with a limited rotation range.  The effective mechanical rotation
 *    range is 14 degrees to 346 degrees, corresponding to the effective
 *    electrical rotation range 0 degrees to 332 degrees.
 *      Angle position (degrees) = (ival - 2048)/4095*333 + 180
 * 2) Dual-sensor mode is mainly used for continuous rotating joshort control
 *    (e.g. a wheel).  The effective rotation range is 0 degrees to 360
 *    degrees.  Dual sensorconfiguration is only available for channel 0
 *    and channel 1.  By connecting two potentiometers to potentiometer
 *    channel 0 and channel 5, and by specifying the sensor type with
 *    command setDcMotorSensorUsage set to "Dual potentiometer sensor"
 *    the channel 0 reading will combine these two sensor readings into
 *    0 degrees to 360 degree range.  For channel 1, channel 1 and channel
 *    4 would be combined instead.
 *      Angle position (degrees) = (ival - 2214)/2214*180 + 180
 *
 * @see setDcMotorSensorUsage
 */
/*
static short getSensorPot(const int *motorSensorData, short channel)
{
    return motorSensorData[2*channel + POTENTIOMETER_SENSOR_OFFSET] + 1 << 8 | motorSensorData[2*channel + POTENTIOMETER_SENSOR_OFFSET];
}
*/

/**
 * Returns the sampling value of the selected motor current sensor.
 *
 * @param channel 0, 1, 2, 3, 4, or 5 for sensor #1, #2, #3, #4, #5, or #6.
 *
 * @return The raw value of the analog to digital converter indicating the
 * motor current.  The data range is between 0 and 4095.  The actual
 * current value can be calculated with the following formula:
 * Motor Current (amperes) = ival/728 ( = ival*3*375/200/4095)
 */
/*
static short getMotorCurrent(const int *motorSensorData, short channel)
{
    return (motorSensorData[2*channel + MOTOR_CURRENT_SENSOR_OFFSET + 1] << 8 | motorSensorData[2*channel + MOTOR_CURRENT_SENSOR_OFFSET]) / 728.0;
}
*/

/**
 * Returns +1, 0, or -1 to indicate the direction of rotation.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first
 * person perspective).
 *
 * @return 1 to indicate positive direction, 0 to indicate no movement,
 * and -1 to indicate negative direction.
 */
/*
static short getEncoderDirection(const int *motorSensorData, short channel)
{
    int offset = channel + ENCODER_DIRECTION_OFFSET;
    short result = -1;
    
    switch (channel)
    {
	case 0:
		result = motorSensorData[offset] & 0x01;
		break;
	case 1:
		result = motorSensorData[offset] & 0x03;
		break;
    }
    
    return result;
}
*/

/**
 * Returns the current pulse counter to indicate the position of rotation.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first
 * person perspective).
 *
 * @return Pulse counter, an short integral value to rotation with range of
 * 0 to 32767 in cycles.
 */
/*
static short getEncoderPulse(const int *motorSensorData, short channel)
{
    int offset = 4*channel + ENCODER_PULSE_OFFSET;
    return motorSensorData[offset + 1] << 8 | motorSensorData[offset];
}
*/

/**
 * Returns the rotation speed.  The unit is defined as the absolute value
 * of the pulse change within 1 second.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first
 * person perspective).
 *
 * @see setDcMotorSensorUsage
 */
/*
static short getEncoderSpeed(const int *motorSensorData, short channel)
{
    int offset = 4*channel + MOTOR_SPEED_OFFSET;
    return motorSensorData[offset + 1] << 8 | motorSensorData[offset];
}
*/

/**
 * Returns the sampling value of the custom analog to digital
 * input signals.  By default, custom AD1 - AD3 are used as the inputs of
 * power supply voltage monitors for DSP circuits, DC motors, and servo
 * motors.  Users may change this setting by configuring the jumpers on
 * the PMS5005->  Please refer to the PMS5005 Robot Sensing and Motion
 * Controller hardware user's manual for detailed information on hardware
 * jumper settings.
 *
 * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8
 *
 * @return The raw value from the analog to digital converter indicating
 * the input voltage levels.  The data range is between 0 and 4095. The
 * voltage levels can be calculated from the following equation:
 * Sensor output voltage = (ival) * 3.0v/4095
 *
 * @see getSensorBatteryAD
 */
/*
static short getCustomAD(const int *customSensorData, short channel)
{
    int offset = 2*channel + CUSTOM_AD_OFFSET;
    return customSensorData[offset + 1] << 8 | customSensorData[offset];
}
*/

/**
 * Returns a value with the lower 8 bits corresponding to the 8 channel
 * custom digital inputs.
 *
 * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8.
 */
/*
static short getCustomDIn(const int *customSensorData, unsigned char channel)
{
    // TODO Auto-generated method stub
    return 0;
}
*/

/**
 * Sets the 8-channel custom digital outputs.
 *
 * @param ival -- only the lower 8 bits are valid and can change the
 * corresponding outputs of the 8 channels.  The MSB of the lower unsigned char
 * represents channel #8 and LSB of the lower unsigned char represents channel #1.
 */
/*
static Buffer * setCustomDOut(Buffer *buf, unsigned char ival)
{
    return NULL;
    // TODO Auto-generated method stub
}
*/

/**
 * Sets the motor polarity to 1 or -1 for the motor channel specified.
 *
 * When the motor is running in the positive dirction, the pontentiometer
 * value is also increasing; motor polarity should be set to 1 (default).
 * When the motor is running in the negative direction, the value is
 * decreasing, and the motor polarity should be set to -1 to change the
 * sensor mounting value so that the potentiometer value increases.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @param polarity 1 or -1
 */
static Buffer * setMotorPolarity(Buffer *buf, unsigned char channel, unsigned char polarity)
{
    assert (buf != NULL);

	buf->length = 12;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PARAM_SET_DID;                   //DID
	buf->data[5] = 5;                           //LEN
	buf->data[6] = DC_SENSOR_USAGE_DID;             //Subcommand
	buf->data[7] = channel;                     //0-L | 1=R
	buf->data[8] = polarity;                    //polarity 1 | -1
	buf->data[9] = calcCRC(buf);                 //Checksum
	buf->data[10] = ETX0;
	buf->data[11] = ETX1;
    
    return buf;
}

/**
 * Enables the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @deprecated
 *
 * @see resumeDCMotor
 */
static Buffer * enableDCMotor(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID; //DID
	buf->data[5] = 2;                //LEN
	buf->data[6] = 1;                //1 = Enable/Resume
	buf->data[7] = channel;          //0=L | 1=R
	buf->data[8] = calcCRC(buf);      //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Disables the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @deprecated
 *
 * @see suspendDCMotor
 */
static Buffer * disableDCMotor(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID;         //DID
	buf->data[5] = 2;                        //LEN
	buf->data[6] = 0;                        //0 = Disable/Suspend
	buf->data[7] = channel;                  //0=L | 1=R
	buf->data[8] = calcCRC(buf);              //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Resumes the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 */
static Buffer * resumeDCMotor(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID; //DID
	buf->data[5] = 2;                //LEN
	buf->data[6] = 1;                //resume
	buf->data[7] = channel;          //0=L | 1=R
	buf->data[8] = calcCRC(buf);      //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Suspends the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * All motor control channels are initially suspended at boot-up.
 */
static Buffer * suspendDCMotor(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID; //DID
	buf->data[5] = 2;                //LEN
	buf->data[6] = 0;                //SUSPEND
	buf->data[7] = channel;          //0=L | 1=R
	buf->data[8] = calcCRC(buf);      //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Sets up the PID control parameters of the specified DC motor channel
 * for position control.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 * @param kp proportional gain (default is 50)
 * @param kd derivative gain (default is 5)
 * @param ki_x100 the desired integral gain * 100.  when Ki_100 = 100,
 * the actual integral control term is Ki = 1.  Ki_x100 has a range of
 * 0 to 25599, where 0 means no integral control (default).
 *
 * @see setDcMotorControlMode
 */
static Buffer * setDCMotorPositionPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100)
{
    assert (buf != NULL);

	buf->length = 20;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PARAM_SET_DID;   //DID
	buf->data[5] = 11;                    //LEN
	buf->data[6] = DC_POSITION_PID_DID;  //Subcommand
	buf->data[7] = channel;                    //0=L | 1=R
	buf->data[8] = KP_ID;                       //Proportional gain
	buf->data[9] = kp;
	buf->data[10] = kp >> 8;
	buf->data[11] = KD_ID;                       //Derivative gain
	buf->data[12] = kd;
	buf->data[13] = kd >> 8;
	buf->data[14] = KI_ID;                       //Integral gain
	buf->data[15] = ki_x100;
	buf->data[16] = ki_x100 >> 8;
	buf->data[17] = calcCRC(buf);                //Checksum
	buf->data[18] = ETX0;
	buf->data[19] = ETX1;
    
    return buf;
}

static Buffer * setDCMotorVelocityPID(Buffer *buf, unsigned char channel, short kp, short kd, short ki_x100)
{
    assert (buf != NULL);

	buf->length = 20;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PARAM_SET_DID;                  //DID
	buf->data[5] = 11;                         //LEN
	buf->data[6] = DC_VELOCITY_PID_DID;            //Subcommand
	buf->data[7] = channel;                    //0=L | 1=R
	buf->data[8] = KP_ID;                       //Proportional gain
	buf->data[9] = kp;
	buf->data[10] = kp >> 8;
	buf->data[11] = KD_ID;                       //Derivative gain
	buf->data[12] = kd;
	buf->data[13] = kd >> 8;
	buf->data[14] = KI_ID;                       //Integral gain
	buf->data[15] = ki_x100;
	buf->data[16] = ki_x100 >> 8;
	buf->data[17] = calcCRC(buf);                //Checksum
	buf->data[18] = ETX0;
	buf->data[19] = ETX1;
    
    return buf;
}

/**
 * This filtering feature is still under development. All data will be
 * treated as raw data.
 */
static Buffer * setDCMotorSensorFilter(Buffer *buf, unsigned char channel, short filterMethod)
{
    return NULL;
}

/**
 * Set the sensor type for the specified DC motor control channel on the
 * Sensing and Motion Controller (PMS5005).
 *
 * The available sensor types are
 * single potentiometer, dual potentiometers, and quadrature encoder.  The
 * single potentiometer sensor is for the control of robot joint with
 * limited rotation range (0 degrees to 332 degrees).  The dual
 * potentiometers and the quadrature sensor are for use with continuously
 * rotating joints (e.g. wheels).
 *
 * @param channel 0, 1, 2, 3, 4, or 5 for single potentiometer sensor
 * channel 0, 1, or 2 for dual potentiometer sensor
 * channel 0 or 1 for quadrature encoder
 * @param sensorType 0 -- single potentiometer sensor
 * 1 -- dual potentiometer sensor
 * 2 -- quadrature encoder
 *
 * Please note
 * 1) The electrical angular range of the potentiometer position sensor is
 *    0 degrees to 332 degrees and the corresponding mechanical rotation
 *    range is 14 degrees to 346 degrees, with the 180 degree position
 *    defined as the sensor's physical middle position.
 * 2) Each DC motor channel for dual potentiometer sensors utilizes two
 *    potentiometer channels.  DC motor channel 0
 *
 * @see getSensorPot
 */
static Buffer * setDCMotorSensorUsage(Buffer *buf, unsigned char channel, unsigned char sensorType)
{
    assert (buf != NULL);

	buf->length = 12;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PARAM_SET_DID;                   //DID
	buf->data[5] = 5;                           //LEN
	buf->data[6] = DC_SENSOR_USAGE_DID;             //Subcommand
	buf->data[7] = channel;                     //0-5  = Single Potentiometer, 0-2  = Dual Potentiometer, 0-1  = Encoder
	buf->data[8] = sensorType;                  //0x00 = Single Potentiometer, 0x01 = Dual Potentiometer, 0x02 = Encoder
	buf->data[9] = calcCRC(buf);                 //Checksum
	buf->data[10] = ETX0;
	buf->data[11] = ETX1;
    
    return buf;
}

/**
 * Sets the control mode of the specified DC motor control channel on the
 * Sensing and Motion Controller (PMS5005).  The available control modes
 * are open-loop PWM control, closed-loop position control, and closed-
 * loop velocity control.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param controlMode 0 == open-loop PWM control, 1 == closed-loop position
 * control, 2 == closed-loop velocity control
 *
 * @see setDcMotorPositionControlPid
 * @see setDcMotorVelocityControlPid
 */
static Buffer * setDCMotorCtrlMode(Buffer *buf, unsigned char channel, unsigned char controlMode)
{
    assert (buf != NULL);

	buf->length = 12;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PARAM_SET_DID;				//DID
	buf->data[5] = 3;						//LEN
	buf->data[6] = DC_CTRL_MODE_DID;			//Subcommand
	buf->data[7] = channel;				//channel 0-5
	buf->data[8] = controlMode;			//0 = open, 1 = closed position, 2 = closed velocity
	buf->data[9] = calcCRC(buf);			//Checksum
	buf->data[10] = ETX0;
	buf->data[11] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to the specified motion control
 * channel on the Sensing and Motion Controller (PMS5005).
 * The command includes the target position and the target time
 * period to execute the command.  The current trajectory planning method
 * with time control is linear.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param pos Target position value
 * @param timePeriod Executing time in milliseconds
 */
static Buffer * setDCMotorPositionWithTime(Buffer *buf, unsigned char channel, short pos, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 14;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = POSITION_CTRL_DID;	//DID
	buf->data[5] = 5;				//LEN
	buf->data[6] = channel;			//Channel 0-5
	buf->data[7] = pos;		//packetValue
	buf->data[8] = pos >> 8;
	buf->data[9] = timePeriod;			//time
	buf->data[10] = timePeriod >> 8;
	buf->data[11] = calcCRC(buf);		       //Checksum
	buf->data[12] = ETX0;
	buf->data[13] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to the specified motion control
 * channel on the Sensing and Motion Controller (PMS5005).  The command
 * includes the target position but no time period specified to execute
 * the command.  The motion controller will drive the motor to the target
 * position at the maximum speed.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param packetValue Target position value
 *
 * 1) Motor will be enabled automatically by the system when this command
 *    is received.
 * 2) No velocity is available for motor channel using single potentiometer
 *    sensor.
 * 3) The unit of the velocity is (Position change in A/D sampling data) /
 *    second when using dual potentiometer sensor for rotational postion
 *    measurement and pulse/second when using quadrature encoder.
 *
 * @see dcMotorVelocityTimeCtrl
 * @see getSensorPot
 */
static Buffer * setDCMotorPosition(Buffer *buf, unsigned char channel, short packetValue)
{
    assert (buf != NULL);

	buf->length = 12;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = POSITION_CTRL_DID;     // DID
	buf->data[5] = 3;			        //LEN
	buf->data[6] = channel;			//channel 0-5
	buf->data[7] = packetValue;		//packetValue
	buf->data[8] = packetValue >> 8;
	buf->data[9] = calcCRC(buf);			//Checksum
	buf->data[10] = ETX0;
	buf->data[11] = ETX1;
    
    return buf;
}

/**
 * Sends the PWM control command to the specified motion control channel on
 * the Sensing and Motion Controller (PMS5005).  The command includes the
 * target pulse width value and the time period to execute the command.
 * The current trajectory planning method for time control is linear.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param packetValue Target pulse width value
 * @param timePeriod Executing time in milliseconds
 *
 * 1) The specified channel (motor) will be enabled automatically by the
 *    system when this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will
 *    cause the motor to turn clockwise (facing the front side of the
 *    motor) and any value in between 0 - 16362 will cause the motor to
 *    turn counter-clockwise.
 *
 * @see dcMotorPwmNonTimeCtrl
 */
static Buffer * setDCMotorPulseWithTime(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 14;
	
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PWM_CTRL_DID;	//DID
	buf->data[5] = 5;			//LEN
	buf->data[6] = channel;		//Channel 0-5
	buf->data[7] = pulseWidth;		//packetValue
	buf->data[8] = pulseWidth >> 8;
	buf->data[9] = timePeriod;			//time
	buf->data[10] = timePeriod >> 8;
	buf->data[11] = calcCRC(buf);					//Checksum
	buf->data[12] = ETX0;
	buf->data[13] = ETX1;
    
    return buf;
}

/**
 * Sends the PWM control command to the specified motion control channel on
 * the Sensing and Motion Controller (PMS5005).  The command includes the
 * target pulse width value without an execution time period specified.
 * The motion controller will set the PWM output of this channel to the
 * target value immediately.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param pulseWidth Target pulse width value
 *
 * 1) The specified channel (motor) will be enabled automatically by the
 *    system when this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the "Stop" stage.  Any value between 16364 - 32767 will cause the
 *    motor to turn clockwise (facing the front side of the motor) and any
 *    value in between 0 - 16362 will cause the motor to turn
 *    counter-clockwise.
 *
 * @see dcMotorPwmTimeCtrl
 */
static Buffer * setDCMotorPulse(Buffer *buf, unsigned char channel, short pulseWidth)
{
    assert (buf != NULL);

	buf->length = 12;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = PWM_CTRL_DID;  //DID
	buf->data[5] = 3;			//LEN
	buf->data[6] = channel;		//Channel 0-5
	buf->data[7] = pulseWidth;		//packetValue
	buf->data[8] = pulseWidth >> 8;
	buf->data[9] = calcCRC(buf);					//Checksum
	buf->data[10] = ETX0;
	buf->data[11] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 DC motor control channels on
 * the sensing and motion controller (PMS5005) at the same time.  The
 * command includes the target positions and the time period to execute the
 * command.  The current trajectory planning method for time control is
 * linear.
 *
 * @param pos1 Target position for channel #1
 * @param pos2 Target position for channel #2
 * @param pos3 Target position for channel #3
 * @param pos4 Target position for channel #4
 * @param pos5 Target position for channel #5
 * @param pos6 Target position for channel #6
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All DC Motors will be enabled automatically by the system when this
 *    command is received.
 * 2) Target position value is the A/D sampling data range 0 to 4095 when
 *    using single potentiometer, 0-4428 when using dual potentiometers.
 * 3) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 4) When using the encoder as sensor input, the target position value is
 *    the pulse count in the range of 0-32767.
 * 5) When omitting motor channels from control, the command value should
 *    be set to -32768 (0x8000), which implies NO_CTRL.
 *
 * @see getSensorPot
 * @see dcMotorPositionTimeCtrl
 */
static Buffer * setAllDCMotorPositionsWithTime(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_POSITION_CTRL_DID; // DID
	buf->data[5] = 14;                            // LEN
	buf->data[6] = pos0;               //channel 1
	buf->data[7] = pos0 >> 8;
	buf->data[8] = pos1;               //channel 2
	buf->data[9] = pos1 >> 8;
	buf->data[10] = pos2;               //channel 3
	buf->data[11] = pos2 >> 8;
	buf->data[12] = pos3;               //channel 4
	buf->data[13] = pos3 >> 8;
	buf->data[14] = pos4;               //channel 5
	buf->data[15] = pos4 >> 8;
	buf->data[16] = pos5;
	buf->data[17] = pos5 >> 8;
	buf->data[18] = timePeriod;				//time
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);                       //Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 DC motor control channels on
 * the Sensing and Motion Controller (PMS5005) at the same time.  The
 * command includes the target positions without a specified time period
 * for execution.  The motion controller will drive the motor to reach the
 * target position with maximum effort.
 *
 * @param pos0 Target position for channel #1
 * @param pos1 Target position for channel #2
 * @param pos2 Target position for channel #3
 * @param pos3 Target position for channel #4
 * @param pos4 Target position for channel #5
 * @param pos5 Target position for channel #6
 *
 * 1) All DC Motors will be enabled automatically by the system when this
 *    command is received.
 * 2) Target position value is the A/D sampling data range 0 to 4095 when
 *    using single potentiometer, 0-4428 when using dual potentiometers.
 * 3) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 4) When using the encoder as sensor input, the target position value is
 *    the pulse count in the range of 0-32767.
 * 5) When omitting motor channels from control, the command value should
 *    be set to -32768 (0x8000), which implies NO_CTRL.
 *
 * @see getSensorPot
 * @see dcMotorPositionNonTimeCtrl
 */
static Buffer * setAllDCMotorPositions(Buffer *buf, short pos0, short pos1, short pos2, short pos3, short pos4, short pos5)
{
    assert (buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_POSITION_CTRL_DID;                 //DID
	buf->data[5] = 12;                                //LEN
	buf->data[6] = pos0;               //channel 1
	buf->data[7] = pos0 >> 8;
	buf->data[8] = pos1;               //channel 2
	buf->data[9] = pos1 >> 8;
	buf->data[10] = pos2;               //channel 3
	buf->data[11] = pos2 >> 8;
	buf->data[12] = pos3;               //channel 4
	buf->data[13] = pos3 >> 8;
	buf->data[14] = pos4;               //channel 5
	buf->data[15] = pos4 >> 8;
	buf->data[16] = pos5;               //channel 6
	buf->data[17] = pos5 >> 8;
	buf->data[18] = calcCRC(buf);                       //Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 DC motor control channels on
 * the sensing and motion controller (PMS5005) at the same time.  The
 * command includes the target positions and the time period to execute the
 * command.  The current trajectory planning method for time control is
 * linear.
 *
 * @param posLeft Target position for channel #1
 * @param posRight Target position for channel #2
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All DC Motors will be enabled automatically by the system when this
 *    command is received.
 * 2) Target position value is the A/D sampling data range 0 to 4095 when
 *    using single potentiometer, 0-4428 when using dual potentiometers.
 * 3) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 4) When using the encoder as sensor input, the target position value is
 *    the pulse count in the range of 0-32767.
 * 5) When omitting motor channels from control, the command value should
 *    be set to -32768 (0x8000), which implies NO_CTRL.
 *
 * @see getSensorPot
 * @see dcMotorPositionTimeCtrl
 */
static Buffer * setBothDCMotorPositionsWithTime(Buffer *buf, short posLeft, short posRight, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_POSITION_CTRL_DID; // DID
	buf->data[5] = 14;                            // LEN
	buf->data[6] = posLeft;               //channel 1
	buf->data[7] = posLeft >> 8;
	buf->data[8] = posRight;               //channel 2
	buf->data[9] = posRight >> 8;
	buf->data[10] = NO_CTRL;               //channel 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;               //channel 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;               //channel 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = timePeriod;				//timePeriod
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);                       //Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 DC motor control channels on
 * the sensing and motion controller (PMS5005) at the same time.  The
 * command includes the target positions and the time period to execute the
 * command.  The current trajectory planning method for time control is
 * linear.
 *
 * @param posLeft Target position for channel #1
 * @param posRight Target position for channel #2
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All DC Motors will be enabled automatically by the system when this
 *    command is received.
 * 2) Target position value is the A/D sampling data range 0 to 4095 when
 *    using single potentiometer, 0-4428 when using dual potentiometers.
 * 3) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 4) When using the encoder as sensor input, the target position value is
 *    the pulse count in the range of 0-32767.
 * 5) When omitting motor channels from control, the command value should
 *    be set to -32768 (0x8000), which implies NO_CTRL.
 *
 * @see getSensorPot
 * @see dcMotorPositionNonTimeCtrl
 */
static Buffer * setBothDCMotorPositions(Buffer *buf, short posLeft, short posRight)
{
    assert (buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_POSITION_CTRL_DID; // DID
	buf->data[5] = 14;                            // LEN
	buf->data[6] = posLeft;               //channel 1
	buf->data[7] = posLeft >> 8;
	buf->data[8] = posRight;               //channel 2
	buf->data[9] = posRight >> 8;
	buf->data[10] = NO_CTRL;               //channel 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;               //channel 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;               //channel 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = calcCRC(buf);                       //Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the velocity control command to all 6 DC motor control channels on
 * the Sensing and Motion Controller (PMS5005) at the same time.  The
 * command includes the target velocities and the time period to execute
 * the command.  The trajectory planning method for time control is linear.
 *
 * @param v0 Target velocity for channel #1
 * @param v1 Target velocity for channel #2
 * @param v2 Target velocity for channel #3
 * @param v3 Target velocity for channel #4
 * @param v4 Target velocity for channel #5
 * @param v5 Target velocity for channel #6
 * @param timePeriod Execution time in milliseconds
 *
 * 1) Motor will be enabled automatically by the system when this command
 *    is received.
 * 2) No velocity control is available for a motor channel operating in
 *    single potentiometer mode.
 * 3) The unit of the velocity is (Position change in A/D sampling data) /
 *    second when using dual potentiometer sensors for rotational position
 *    measurements and pulse/second when using quadrature encoder.
 * 4) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 5) When omitting motors from control, send the command value -32768
 *    (0x8000), which implies NO_CTRL.
 *
 * @see dcMotorVelocityTimeCtrl
 */
static Buffer * setAllDCMotorVelocitiesWithTime(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_VELOCITY_CTRL_DID;			//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = v0;		//MOTOR 1
	buf->data[7] = v0 >> 8;
	buf->data[8] = v1;		//MOTOR 2
	buf->data[9] = v1 >> 8;
	buf->data[10] = v2;		//MOTOR 3
	buf->data[11] = v2 >> 8;
	buf->data[12] = v3;		//MOTOR 4
	buf->data[13] = v3 >> 8;
	buf->data[14] = v4;		//MOTOR 5
	buf->data[15] = v4 >> 8;
	buf->data[16] = v5;		//MOTOR 6
	buf->data[17] = v5 >> 8;
	buf->data[18] = timePeriod;		//timePeriod
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);				//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the velocity control command to all 6 DC motor control channels on
 * the Sensing and Motion Controller (PMS5005) at the same time.  The
 * command includes the target velocities without specifying an execution
 * time period.  The motion controller will drive the motor to achieve
 * the target velocity with maximum effort.
 *
 * @param pos1 Target velocity for channel #1
 * @param pos2 Target velocity for channel #2
 * @param pos3 Target velocity for channel #3
 * @param pos4 Target velocity for channel #4
 * @param pos5 Target velocity for channel #5
 * @param pos6 Target velocity for channel #6
 *
 * 1) Motor will be enabled automatically by the system when this command
 *    is received.
 * 2) No velocity control is available for a motor channel operating in
 *    single potentiometer mode.
 * 3) The unit of the velocity is (Position change in A/D sampling data) /
 *    second when using dual potentiometer sensors for rotational position
 *    measurements and pulse/second when using quadrature encoder.
 * 4) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 5) When omitting motors from control, send the command value -32768
 *    (0x8000), which implies NO_CTRL.
 *
 * @see dcMotorVelocityNonTimeCtrl
 */
static Buffer * setAllDCMotorVelocities(Buffer *buf, short v0, short v1, short v2, short v3, short v4, short v5)
{
    assert (buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_VELOCITY_CTRL_DID;			//DID
	buf->data[5] = 12;						//LEN
	buf->data[6] = v0;		//MOTOR 1
	buf->data[7] = v0 >> 8;
	buf->data[8] = v1;		//MOTOR 2
	buf->data[9] = v1 >> 8;
	buf->data[10] = v2;		//MOTOR 3
	buf->data[11] = v2 >> 8;
	buf->data[12] = v3;		//MOTOR 4
	buf->data[13] = v3 >> 8;
	buf->data[14] = v4;		//MOTOR 5
	buf->data[15] = v4 >> 8;
	buf->data[16] = v5;		//MOTOR 6
	buf->data[17] = v5 >> 8;
	buf->data[18] = calcCRC(buf);				//Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the velocity control command to all 6 DC motor control channels on
 * the Sensing and Motion Controller (PMS5005) at the same time.  The
 * command includes the target velocities and the time period to execute
 * the command.  The trajectory planning method for time control is linear.
 *
 * @param vLeft Target velocity for channel #1
 * @param vRight Target velocity for channel #2
 * @param timePeriod Execution time in milliseconds
 *
 * 1) Motor will be enabled automatically by the system when this command
 *    is received.
 * 2) No velocity control is available for a motor channel operating in
 *    single potentiometer mode.
 * 3) The unit of the velocity is (Position change in A/D sampling data) /
 *    second when using dual potentiometer sensors for rotational position
 *    measurements and pulse/second when using quadrature encoder.
 * 4) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 5) When omitting motors from control, send the command value -32768
 *    (0x8000), which implies NO_CTRL.
 *
 * @see dcMotorVelocityTimeCtrl
 */
static Buffer * setBothDCMotorVelocitiesWithTime(Buffer *buf, short vLeft, short vRight, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_VELOCITY_CTRL_DID;			//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = vLeft;		//MOTOR 1
	buf->data[7] = vLeft >> 8;
	buf->data[8] = vRight;		//MOTOR 2
	buf->data[9] = vRight >> 8;
	buf->data[10] = NO_CTRL;		//MOTOR 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;		//MOTOR 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;		//MOTOR 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;		//MOTOR 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = timePeriod;		//time
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);				//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the velocity control command to all 6 DC motor control channels on
 * the Sensing and Motion Controller (PMS5005) at the same time.  The
 * command includes the target velocities and the time period to execute
 * the command.  The trajectory planning method for time control is linear.
 *
 * @param leftVelocity Target velocity for channel #1
 * @param rightVelocity Target velocity for channel #2
 *
 * 1) Motor will be enabled automatically by the system when this command
 *    is received.
 * 2) No velocity control is available for a motor channel operating in
 *    single potentiometer mode.
 * 3) The unit of the velocity is (Position change in A/D sampling data) /
 *    second when using dual potentiometer sensors for rotational position
 *    measurements and pulse/second when using quadrature encoder.
 * 4) Please refer to the description of getSensorPot for data conversion
 *    between angular values and the A/D sampling data values.
 * 5) When omitting motors from control, send the command value -32768
 *    (0x8000), which implies NO_CTRL.
 *
 * @see dcMotorVelocityTimeCtrl
 */
static Buffer * setBothDCMotorVelocities(Buffer *buf, short vLeft, short vRight)
{
    assert (buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_VELOCITY_CTRL_DID;			//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = vLeft;		//MOTOR 1
	buf->data[7] = vLeft >> 8;
	buf->data[8] = vRight;		//MOTOR 2
	buf->data[9] = vRight >> 8;
	buf->data[10] = NO_CTRL;		//MOTOR 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;		//MOTOR 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;		//MOTOR 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;		//MOTOR 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = calcCRC(buf);				//Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the PWM control command to all 6 DC motor control channels on the
 * Sensing and Motion Controller (PMS5005) at the same time.  The command
 * includes the target PWM values and the time period for execution.  The
 * current trajectory planning method for time control is linear.
 *
 * @param p0 Target PWM value for channel #1
 * @param p1 Target PWM value for channel #2
 * @param p2 Target PWM value for channel #3
 * @param p3 Target PWM value for channel #4
 * @param p4 Target PWM value for channel #5
 * @param p5 Target PWM value for channel #6
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All channels (motors) will be enable automatically by the system when
 *    this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will
 *    cause the motor to turn clockwise (facing the front side of the
 *    motor) and any value in between 0 - 16362 will cause the motor to
 *    turn counter-clockwise.
 * 4) When omitting motors from control, the command value of -32768
 *    (0x8000), should be sent.  This implies NO_CTRL.
 */
static Buffer * setAllDCMotorPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_PWM_CTRL_DID;				//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = p0;		//MOTOR 1
	buf->data[7] = p0 >> 8;
	buf->data[8] = p1;		//MOTOR 2
	buf->data[9] = p1 >> 8;
	buf->data[10] = p2;		//MOTOR 3
	buf->data[11] = p2 >> 8;
	buf->data[12] = p3;		//MOTOR 4
	buf->data[13] = p3 >> 8;
	buf->data[14] = p4;		//MOTOR 5
	buf->data[15] = p4 >> 8;
	buf->data[16] = p5;		//MOTOR 6
	buf->data[17] = p5 >> 8;
	buf->data[18] = timePeriod;		//timePeriod
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);				//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;

	return buf;
}

/**
 * Sends the PWM control command to all 6 DC motor control channels on the
 * Sensing and Motion Controller (PMS5005) at the same time.  The command
 * includes the target PWM values without a specified time period for
 * execution.  The motion controller will adjust the pulse width right away.
 *
 * @param p0 Target PWM value for channel #1
 * @param p1 Target PWM value for channel #2
 * @param p2 Target PWM value for channel #3
 * @param p3 Target PWM value for channel #4
 * @param p4 Target PWM value for channel #5
 * @param p5 Target PWM value for channel #6
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All channels (motors) will be enable automatically by the system when
 *    this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will
 *    cause the motor to turn clockwise (facing the front side of the
 *    motor) and any value in between 0 - 16362 will cause the motor to
 *    turn counter-clockwise.
 * 4) When omitting motors from control, the command value of -32768
 *    (0x8000), should be sent.  This implies NO_CTRL.
 */
static Buffer * setAllDCMotorPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5)
{
    assert (buf != NULL);

	buf->length = 21;

	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_PWM_CTRL_DID;				//DID
	buf->data[5] = 12;						//LEN
	buf->data[6] = p0;		//MOTOR 1
	buf->data[7] = p0 >> 8;
	buf->data[8] = p1;		//MOTOR 2
	buf->data[9] = p1 >> 8;
	buf->data[10] = p2;		//MOTOR 3
	buf->data[11] = p2 >> 8;
	buf->data[12] = p3;		//MOTOR 4
	buf->data[13] = p3 >> 8;
	buf->data[14] = p4;		//MOTOR 5
	buf->data[15] = p4 >> 8;
	buf->data[16] = p5;		//MOTOR 6
	buf->data[17] = p5 >> 8;
	buf->data[18] = calcCRC(buf);				//Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the PWM control command to all 6 DC motor control channels on the
 * Sensing and Motion Controller (PMS5005) at the same time.  The command
 * includes the target PWM values and the time period for execution.  The
 * current trajectory planning method for time control is linear.
 *
 * @param pLeft Target PWM value for channel #1
 * @param pRight Target PWM value for channel #2
 * @param timePeriod Execution time in milliseconds
 *
 * 1) All channels (motors) will be enable automatically by the system when
 *    this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will
 *    cause the motor to turn clockwise (facing the front side of the
 *    motor) and any value in between 0 - 16362 will cause the motor to
 *    turn counter-clockwise.
 * 4) When omitting motors from control, the command value of -32768
 *    (0x8000), should be sent.  This implies NO_CTRL.
 */
static Buffer * setBothDCMotorPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_PWM_CTRL_DID;				//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = pLeft;		//MOTOR 1
	buf->data[7] = pLeft >> 8;
	buf->data[8] = pRight;		//MOTOR 2
	buf->data[9] = pRight >> 8;
	buf->data[10] = NO_CTRL;		//MOTOR 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;		//MOTOR 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;		//MOTOR 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;		//MOTOR 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = timePeriod;		//time
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);				//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the PWM control command to all 6 DC motor control channels on the
 * Sensing and Motion Controller (PMS5005) at the same time.  The command
 * includes the target PWM values and the time period for execution.  The
 * current trajectory planning method for time control is linear.
 *
 * @param pLeft Target PWM value for channel #1
 * @param pRight Target PWM value for channel #2
 *
 * 1) All channels (motors) will be enable automatically by the system when
 *    this command is received.
 * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding
 *    to the duty cycle of 0 to 100% linearly.
 * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor
 *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will
 *    cause the motor to turn clockwise (facing the front side of the
 *    motor) and any value in between 0 - 16362 will cause the motor to
 *    turn counter-clockwise.
 * 4) When omitting motors from control, the command value of -32768
 *    (0x8000), should be sent.  This implies NO_CTRL.
 */
static Buffer * setBothDCMotorPulses(Buffer *buf, short pLeft, short pRight)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_PWM_CTRL_DID;				//DID
	buf->data[5] = 14;						//LEN
	buf->data[6] = pLeft;		//MOTOR 1
	buf->data[7] = pLeft >> 8;
	buf->data[8] = pRight;		//MOTOR 2
	buf->data[9] = pRight >> 8;
	buf->data[10] = NO_CTRL;		//MOTOR 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;		//MOTOR 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;		//MOTOR 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;		//MOTOR 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = calcCRC(buf);				//Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Enables the specified servo motor control channel.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 *
 * All servo motor channels are disable initially at system startup.  They
 * need to be enabled explicitly before use.
 *
 * @see disableServo
 */
static Buffer * enableServo(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID;         //DID
	buf->data[5] = 2;                        //LEN
	buf->data[6] = 0;                        //0 = Enable
	buf->data[7] = channel;                  //6-11 SERVO
	buf->data[8] = calcCRC(buf);              //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Disables the specified servo motor control channel.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 *
 * All servo motor channels are disable initially at system startup.  They
 * need to be enabled explicitly before use.
 *
 * @see enableServo
 */
static Buffer * disableServo(Buffer *buf, unsigned char channel)
{
    assert (buf != NULL);

	buf->length = 11;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = TOGGLE_DC_MOTORS_DID;         //DID
	buf->data[5] = 2;                        //LEN
	buf->data[6] = 0;                        //0 = Disable
	buf->data[7] = channel;                  //6-11 = SERVO
	buf->data[8] = calcCRC(buf);              //Checksum
	buf->data[9] = ETX0;
	buf->data[10] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to the specified servo motor control
 * channel on the Sensing and Motion Controller (PMS5005).  The command
 * includes the target position command and the time period to execute the
 * command.  The current trajectory planning method for time control is
 * linear.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param packetValue Target Pulse Width (in milliseconds) * 2250
 * @param timePeriod Executing time in milliseconds
 *
 * Usually, a standard remote control servo motor expects to get the
 * specified pulse width every 20 seconds in order to hold the
 * corresponding angle position.  The pulse width value in milliseconds
 * for 0 degrees, 90 degrees, and 180 degrees are servo manufacturer and
 * model dependent.  They are around 1ms, 1.5ms, and 2.0ms respectively for
 * most common servos.  Experiments are required to obtain the exact value
 * for a specific servo motor.
 *
 * @see servoNonTimeCtrl
 */
static Buffer * setServoPulseWithTime(Buffer *buf, unsigned char channel, short pulseWidth, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 15;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = SERVO_CTRL_DID;                        //DID
	buf->data[5] = 6;                                 //LEN
	buf->data[6] = channel;                           //channel
	buf->data[7] = pulseWidth;           //command value low 8 bit
	buf->data[8] = pulseWidth >> 8;   //high 8 bit
	buf->data[9] = 6;                                 //flag
	buf->data[10] = timePeriod;               //time low 8 bit
	buf->data[11] = timePeriod >> 8;       //high 8 bit
	buf->data[12] = calcCRC(buf);                       //Checksum
	buf->data[13] = ETX0;
	buf->data[14] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to the specified servo motor control
 * channel on the Sensing and Motion Controller (PMS5005).  The command
 * includes the target position command without a specific time period for
 * execution.  The motion controller will send the desired pulse width to
 * the servo motor right away.
 *
 * @param channel 0, 1, 2, 3, 4, or 5
 * @param packetValue Target pulse width (ms) * 2250
 *
 * @see servoTimeCtrl
 */
static Buffer * setServoPulse(Buffer *buf, unsigned char channel, short pulseWidth)
{
    assert (buf != NULL);

	buf->length = 13;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = SERVO_CTRL_DID;                        //DID
	buf->data[5] = 4;                                 //LEN
	buf->data[6] = channel;                           //channel
	buf->data[7] = pulseWidth;           //command value low 8 bit
	buf->data[8] = pulseWidth >> 8;   //high 8 bit
	buf->data[9] = 6;                                 //flag
	buf->data[10] = calcCRC(buf);                       //Checksum
	buf->data[11] = ETX0;
	buf->data[12] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 servo motor
 * control channels on the Sensing and Motion Controller (PMS5005) at the
 * same time.
 *
 * The command includes the target position commands and the
 * time period to execute the command.  The current trajectory planning
 * method for time control is linear.
 *
 * @param pos1 Target position for channel #1 (Left Motor on X80Pro)
 * @param pos2 Target position for channel #2 (-Right Motor on X80Pro)
 * @param pos3 Target position for channel #3 (NO_CTRL on X80Pro)
 * @param pos4 Target position for channel #4 (NO_CTRL on X80Pro)
 * @param pos5 Target position for channel #5 (NO_CTRL on X80Pro)
 * @param pos6 Target position for channel #6 (NO_CTRL on X80Pro)
 * @param timePeriod Executing time in milliseconds
 *
 * When omitting servo motors from control, please send the command value
 * -32768 (0x8000), which implies NO_CTRL.
 *
 * @see servoTimeCtrl
 */
static Buffer * setAllServoPulsesWithTime(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_SERVO_CTRL_DID;                        //DID
	buf->data[5] = 14;                                    //LEN
	buf->data[6] = p0;				    //channel 1
	buf->data[7] = p0 >> 8;
	buf->data[8] = p1;					//channel 2
	buf->data[9] = p1 >> 8;
	buf->data[10] = p2;					//channel 3
	buf->data[11] = p2 >> 8;
	buf->data[12] = p3;					//channel 4
	buf->data[13] = p3 >> 8;
	buf->data[14] = p4;					//channel 5
	buf->data[15] = p4 >> 8;
	buf->data[16] = p5;					//channel 6
	buf->data[17] = p5 >> 8;
	buf->data[18] = timePeriod;					//timePeriod
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);							//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 servo motor
 * control channels on the Sensing and Motion Controller (PMS5005) at the
 * same time.
 *
 * The command includes the target position commands without specifying a
 * time period in which to execute the command.  The motion controller
 * sends the desired pulse width to the servo motor right away.
 *
 * @param p0 Target position for channel #1 (Left Motor on X80Pro)
 * @param p1 Target position for channel #2 (-Right Motor on X80Pro)
 * @param p2 Target position for channel #3 (NO_CTRL on X80Pro)
 * @param p3 Target position for channel #4 (NO_CTRL on X80Pro)
 * @param p4 Target position for channel #5 (NO_CTRL on X80Pro)
 * @param p5 Target position for channel #6 (NO_CTRL on X80Pro)
 *
 * When omitting servo motors from control, please send the command value
 * -32768 (0x8000), which implies NO_CTRL.
 *
 * @see servoNonTimeCtrl
 */
static Buffer * setAllServoPulses(Buffer *buf, short p0, short p1, short p2, short p3, short p4, short p5)
{
	assert(buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_SERVO_CTRL_DID;                //DID
	buf->data[5] = 12;                            //LEN
	buf->data[6] = p0;           //motor 1
	buf->data[7] = p0 >> 8;
	buf->data[8] = p1;           //motor 2
	buf->data[9] = p1 >> 8;
	buf->data[10] = p2;           //motor 3
	buf->data[11] = p2 >> 8;
	buf->data[12] = p3;           //motor 4
	buf->data[13] = p3 >> 8;
	buf->data[14] = p4;           //motor 5
	buf->data[15] = p4 >> 8;
	buf->data[16] = p5;           //motor 6
	buf->data[17] = p5 >> 8;
	buf->data[18] = calcCRC(buf);                   //Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 servo motor
 * control channels on the Sensing and Motion Controller (PMS5005) at the
 * same time.
 *
 * The command includes the target position commands and the
 * time period to execute the command.  The current trajectory planning
 * method for time control is linear.
 *
 * @param pos1 Target position for channel #1 (Left Motor on X80Pro)
 * @param pos2 Target position for channel #2 (-Right Motor on X80Pro)
 * @param pos3 Target position for channel #3 (NO_CTRL on X80Pro)
 * @param pos4 Target position for channel #4 (NO_CTRL on X80Pro)
 * @param pos5 Target position for channel #5 (NO_CTRL on X80Pro)
 * @param pos6 Target position for channel #6 (NO_CTRL on X80Pro)
 * @param timePeriod Executing time in milliseconds
 *
 * When omitting servo motors from control, please send the command value
 * -32768 (0x8000), which implies NO_CTRL.
 *
 * @see servoTimeCtrl
 */
static Buffer * setBothServoPulsesWithTime(Buffer *buf, short pLeft, short pRight, short timePeriod)
{
    assert (buf != NULL);

	buf->length = 23;
		
	buf->data[0]  = STX0;
	buf->data[1]  = STX1;
	buf->data[2]  = 1;
	buf->data[3]  = 0;
	buf->data[4]  = ALL_SERVO_CTRL_DID;                        //DID
	buf->data[5]  = 14;                                    //LEN
	buf->data[6]  = pLeft;				    //channel 1
	buf->data[7]  = pLeft >> 8;
	buf->data[8]  = pRight;					//channel 2
	buf->data[9]  = pRight >> 8;
	buf->data[10] = NO_CTRL;					//channel 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;					//channel 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;					//channel 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;					//channel 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = timePeriod;					//time
	buf->data[19] = timePeriod >> 8;
	buf->data[20] = calcCRC(buf);							//Checksum
	buf->data[21] = ETX0;
	buf->data[22] = ETX1;
    
    return buf;
}

/**
 * Sends the position control command to all 6 servo motor
 * control channels on the Sensing and Motion Controller (PMS5005) at the
 * same time.
 *
 * The command includes the target position commands without specifying a
 * time period in which to execute the command.  The motion controller
 * sends the desired pulse width to the servo motor right away.
 *
 * @param p0 Target position for channel #1 (Left Motor on X80Pro)
 * @param p1 Target position for channel #2 (-Right Motor on X80Pro)
 *
 * When omitting servo motors from control, please send the command value
 * -32768 (0x8000), which implies NO_CTRL.
 *
 * @see servoNonTimeCtrl
 */
static Buffer * setBothServoPulses(Buffer *buf, short pLeft, short pRight)
{
    assert (buf != NULL);

	buf->length = 21;
		
	buf->data[0] = STX0;
	buf->data[1] = STX1;
	buf->data[2] = 1;
	buf->data[3] = 0;
	buf->data[4] = ALL_SERVO_CTRL_DID;                //DID
	buf->data[5] = 12;                            //LEN
	buf->data[6] = pLeft;           //motor 1
	buf->data[7] = pLeft >> 8;
	buf->data[8] = pRight;           //motor 2
	buf->data[9] = pRight >> 8;
	buf->data[10] = NO_CTRL;           //motor 3
	buf->data[11] = NO_CTRL >> 8;
	buf->data[12] = NO_CTRL;           //motor 4
	buf->data[13] = NO_CTRL >> 8;
	buf->data[14] = NO_CTRL;           //motor 5
	buf->data[15] = NO_CTRL >> 8;
	buf->data[16] = NO_CTRL;           //motor 6
	buf->data[17] = NO_CTRL >> 8;
	buf->data[18] = calcCRC(buf);                   //Checksum
	buf->data[19] = ETX0;
	buf->data[20] = ETX1;
    
    return buf;
}

/**
 * Displays the image data in the file bmpFileName (BMP format) on the
 * graphic LCD connected to the Sensing and Motion Controller (PMS5005).
 *
 * @param bmpFileName Full path of the BMP file for displaying
 *
 * The graphic LCD display is monochrome with dimensions 128 by 64 pixels.
 * The bmp image must be 128x64 pixels in mono.
 */
static Buffer * setLCDDisplayPMS(Buffer *buf, char *bmpFileName)
{
    return NULL;
    // TODO Auto-generated method stub
}

void PMS5005_init(PMS5005_t *self)
{
    self->HEADER_LENGTH = HEADER_LENGTH;
    self->PAYLOAD_OFFSET = PAYLOAD_OFFSET;
    self->DID_OFFSET = DID_OFFSET;
    self->MOTOR_SENSOR_REQUEST_SIZE = MOTOR_SENSOR_REQUEST_SIZE;
    self->STANDARD_SENSOR_REQUEST_SIZE = STANDARD_SENSOR_REQUEST_SIZE;
    self->CUSTOM_SENSOR_REQUEST_SIZE = CUSTOM_SENSOR_REQUEST_SIZE;
    self->ALL_SENSOR_REQUEST_SIZE = ALL_SENSOR_REQUEST_SIZE;
    self->ENABLE_ALL_SENSOR_SENDING_SIZE = ENABLE_ALL_SENSOR_SENDING_SIZE;
    self->ENABLE_STANDARD_SENSOR_SENDING_SIZE = ENABLE_STANDARD_SENSOR_SENDING_SIZE;
    self->ENABLE_CUSTOM_SENSOR_SENDING_SIZE = ENABLE_CUSTOM_SENSOR_SENDING_SIZE;
    self->DISABLE_MOTOR_SENSOR_SENDING_SIZE = DISABLE_MOTOR_SENSOR_SENDING_SIZE;
    self->DISABLE_STANDARD_SENSOR_SENDING_SIZE = DISABLE_STANDARD_SENSOR_SENDING_SIZE;
    self->DISABLE_ALL_SENSOR_SENDING_SIZE = DISABLE_ALL_SENSOR_SENDING_SIZE;
    self->SET_MOTOR_POLARITY_SIZE = SET_MOTOR_POLARITY_SIZE;
    self->ENABLE_DC_MOTOR_SIZE = ENABLE_DC_MOTOR_SIZE;
    self->DISABLE_DC_MOTOR_SIZE = DISABLE_DC_MOTOR_SIZE;
    self->RESUME_DC_MOTOR_SIZE = RESUME_DC_MOTOR_SIZE;
    self->SUSPEND_DC_MOTOR_SIZE = SUSPEND_DC_MOTOR_SIZE;
    self->SET_DC_MOTOR_POSITION_CTRL_PID_SIZE = SET_DC_MOTOR_POSITION_CTRL_PID_SIZE;
    self->SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE = SET_DC_MOTOR_VELOCITY_CTRL_PID_SIZE;
    self->SET_DC_MOTOR_SENSOR_FILTER_SIZE = SET_DC_MOTOR_SENSOR_FILTER_SIZE;
    self->SET_DC_MOTOR_CTRL_MODE_SIZE = SET_DC_MOTOR_CTRL_MODE_SIZE;
    self->DC_MOTOR_POSITION_TIME_CTRL_SIZE = DC_MOTOR_POSITION_TIME_CTRL_SIZE;
    self->DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE;
    self->DC_MOTOR_PWM_TIME_CTRL_SIZE = DC_MOTOR_PWM_TIME_CTRL_SIZE;
    self->DC_MOTOR_PWM_NON_TIME_CTRL_SIZE = DC_MOTOR_PWM_NON_TIME_CTRL_SIZE;
    self->DC_MOTOR_POSITION_TIME_CTRL_SIZE = DC_MOTOR_POSITION_TIME_CTRL_SIZE;
    self->DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE = DC_MOTOR_POSITION_NON_TIME_CTRL_SIZE;
    self->DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE = DC_MOTOR_VELOCITY_TIME_CTRL_ALL_SIZE;
    self->DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE = DC_MOTOR_VELOCITY_NON_TIME_CTRL_ALL_SIZE;
    self->DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE = DC_MOTOR_PWM_TIME_CTRL_ALL_SIZE;
    self->DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE = DC_MOTOR_PWM_NON_TIME_CTRL_ALL_SIZE;
    self->ENABLE_SERVO_SIZE = ENABLE_SERVO_SIZE;
    self->DISABLE_SERVO_SIZE = DISABLE_SERVO_SIZE;
    self->SERVO_TIME_CTRL_SIZE = SERVO_TIME_CTRL_SIZE;
    self->SERVO_NON_TIME_CTRL_SIZE = SERVO_NON_TIME_CTRL_SIZE;
    self->SERVO_TIME_CTRL_ALL_SIZE = SERVO_TIME_CTRL_ALL_SIZE;
    self->SERVO_NON_TIME_CTRL_ALL_SIZE = SERVO_NON_TIME_CTRL_ALL_SIZE;
    self->STX0 = STX0;
    self->STX1 = STX1;
    self->ETX0 = ETX0;
    self->ETX1 = ETX1;
    self->POSITION_CTRL_DID = POSITION_CTRL_DID;
    self->ALL_POSITION_CTRL_DID = ALL_POSITION_CTRL_DID;
    self->PWM_CTRL_DID = PWM_CTRL_DID;
    self->ALL_PWM_CTRL_DID = ALL_PWM_CTRL_DID;
    self->PARAM_SET_DID = PARAM_SET_DID;
    self->DC_POSITION_PID_DID = DC_POSITION_PID_DID;
    self->DC_VELOCITY_PID_DID = DC_VELOCITY_PID_DID;
    self->DC_SENSOR_USAGE_DID = DC_SENSOR_USAGE_DID;
    self->DC_CTRL_MODE_DID = DC_CTRL_MODE_DID;
    self->POWER_CTRL_DID = POWER_CTRL_DID;
    self->LCD_CTRL_DID = LCD_CTRL_DID;
    self->VELOCITY_CTRL_DID = VELOCITY_CTRL_DID;
    self->ALL_VELOCITY_CTRL_DID = ALL_VELOCITY_CTRL_DID;
    self->SERVO_CTRL_DID = SERVO_CTRL_DID;
    self->ALL_SERVO_CTRL_DID = ALL_SERVO_CTRL_DID;
    self->TOGGLE_DC_MOTORS_DID = TOGGLE_DC_MOTORS_DID;
    self->CONSTELLATION_CTRL_DID = CONSTELLATION_CTRL_DID;
    self->GET_MOTOR_SENSOR_DATA_DID = GET_MOTOR_SENSOR_DATA_DID;
    self->GET_CUSTOM_SENSOR_DATA_DID = GET_CUSTOM_SENSOR_DATA_DID;
    self->GET_STANDARD_SENSOR_DATA_DID = GET_STANDARD_SENSOR_DATA_DID;
    self->GET_ALL_SENSOR_DATA_DID = GET_ALL_SENSOR_DATA_DID;
    self->SETUP_COM = SETUP_COM;
    self->PWM_CTRL_MODE = PWM_CTRL_MODE;
    self->POSITION_CTRL_MODE = POSITION_CTRL_MODE;
    self->VELOCITY_CTRL_MODE = VELOCITY_CTRL_MODE;
    self->KP_ID = KP_ID;
    self->KD_ID = KD_ID;
    self->KI_ID = KI_ID;
    self->NON_CTRL_PACKET = NON_CTRL_PACKET;
    self->NO_CTRL = NO_CTRL;
    self->ULTRASONIC_OFFSET = ULTRASONIC_OFFSET;
    self->ENCODER_PULSE_OFFSET = ENCODER_PULSE_OFFSET;
    self->ENCODER_SPEED_OFFSET = ENCODER_SPEED_OFFSET;
    self->STANDARD_IR_RANGE_OFFSET = STANDARD_IR_RANGE_OFFSET;
    self->CUSTOM_IR_RANGE_OFFSET = CUSTOM_IR_RANGE_OFFSET;
    self->HUMAN_ALARM_OFFSET = HUMAN_ALARM_OFFSET;
    self->HUMAN_MOTION_OFFSET = HUMAN_MOTION_OFFSET;
    self->TILTING_X_OFFSET = TILTING_X_OFFSET;
    self->TILTING_Y_OFFSET = TILTING_Y_OFFSET;
    self->ENCODER_DIRECTION_OFFSET = ENCODER_DIRECTION_OFFSET;
    self->MOTOR_SPEED_OFFSET = MOTOR_SPEED_OFFSET;
    self->CUSTOM_AD_OFFSET = CUSTOM_AD_OFFSET;
    self->TEMPERATURE_AD_OFFSET = TEMPERATURE_AD_OFFSET;
    self->OVERHEAT_SENSOR_OFFSET = OVERHEAT_SENSOR_OFFSET;
    self->IR_COMMAND_OFFSET = IR_COMMAND_OFFSET;
    self->BATTERY_SENSOR_OFFSET = BATTERY_SENSOR_OFFSET;
    self->REFERENCE_VOLTAGE_OFFSET = REFERENCE_VOLTAGE_OFFSET;
    self->POTENTIOMETER_POWER_OFFSET = POTENTIOMETER_POWER_OFFSET;
    self->POTENTIOMETER_SENSOR_OFFSET = POTENTIOMETER_SENSOR_OFFSET;
    self->MOTOR_CURRENT_SENSOR_OFFSET = MOTOR_CURRENT_SENSOR_OFFSET;
	self->MOTOR_SENSOR_DATA_LENGTH = MOTOR_SENSOR_DATA_LENGTH;
	self->CUSTOM_SENSOR_DATA_LENGTH = CUSTOM_SENSOR_DATA_LENGTH;
	self->STANDARD_SENSOR_DATA_LENGTH = STANDARD_SENSOR_DATA_LENGTH;
    
    /* methods */
	self->test = test;
    self->calcCRC = calcCRC;
    self->motorSensorRequest = motorSensorRequest;
    self->standardSensorRequest = standardSensorRequest;
    self->customSensorRequest = customSensorRequest;
    self->allSensorRequest = allSensorRequest;
    self->enableMotorSensorSending = enableMotorSensorSending;
    self->enableStandardSensorSending = enableStandardSensorSending;
    self->enableCustomSensorSending = enableCustomSensorSending;
    self->enableAllSensorSending = enableAllSensorSending;
    self->disableMotorSensorSending = disableMotorSensorSending;
    self->disableStandardSensorSending = disableStandardSensorSending;
    self->disableCustomSensorSending = disableCustomSensorSending;
    self->disableAllSensorSending = disableAllSensorSending;
    self->setMotorSensorPeriod = setMotorSensorPeriod;
    self->setStandardSensorPeriod = setStandardSensorPeriod;
    self->setCustomSensorPeriod = setCustomSensorPeriod;
    self->setAllSensorPeriod = setAllSensorPeriod;
    self->setIRCtrlOutput = setIRCtrlOutput;
    self->setCustomDOut = setCustomDOut;
    self->setMotorPolarity = setMotorPolarity;
    self->enableDCMotor = enableDCMotor;
    self->disableDCMotor = disableDCMotor;
    self->resumeDCMotor = resumeDCMotor;
    self->suspendDCMotor = suspendDCMotor;
    self->setDCMotorPositionPID = setDCMotorPositionPID;
    self->setDCMotorVelocityPID = setDCMotorVelocityPID;
    self->setDCMotorSensorFilter = setDCMotorSensorFilter;
    self->setDCMotorSensorUsage = setDCMotorSensorUsage;
    self->setDCMotorCtrlMode = setDCMotorCtrlMode;
    self->setDCMotorPositionWithTime = setDCMotorPositionWithTime;
    self->setDCMotorPosition = setDCMotorPosition;
    self->setDCMotorPulseWithTime = setDCMotorPulseWithTime;
    self->setDCMotorPulse = setDCMotorPulse;
    self->setAllDCMotorPositionsWithTime = setAllDCMotorPositionsWithTime;
    self->setAllDCMotorPositions = setAllDCMotorPositions;
    self->setAllDCMotorVelocitiesWithTime = setAllDCMotorVelocitiesWithTime;
    self->setAllDCMotorVelocities = setAllDCMotorVelocities;
    self->setAllDCMotorPulsesWithTime = setAllDCMotorPulsesWithTime;
    self->setAllDCMotorPulses = setAllDCMotorPulses;
    self->setBothDCMotorPositionsWithTime = setBothDCMotorPositionsWithTime;
    self->setBothDCMotorPositions = setBothDCMotorPositions;
    self->setBothDCMotorVelocitiesWithTime = setBothDCMotorVelocitiesWithTime;
    self->setBothDCMotorVelocities = setBothDCMotorVelocities;
    self->setBothDCMotorPulsesWithTime = setBothDCMotorPulsesWithTime;
    self->setBothDCMotorPulses = setBothDCMotorPulses;
    self->enableServo = enableServo;
    self->disableServo = disableServo;
    self->setServoPulseWithTime = setServoPulseWithTime;
    self->setServoPulse = setServoPulse;
    self->setAllServoPulsesWithTime = setAllServoPulsesWithTime;
    self->setAllServoPulses = setAllServoPulses;
    self->setBothServoPulsesWithTime = setBothServoPulsesWithTime;
    self->setBothServoPulses = setBothServoPulses;
    self->setLCDDisplayPMS = setLCDDisplayPMS;
}
