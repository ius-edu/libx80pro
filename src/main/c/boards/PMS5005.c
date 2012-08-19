#include <Buffer.h>

/* 
 * C implmentation written by Jesse Riddle and Colton Jenkins, with 
 * significant documentation taken from the DrRobot WiRobot SDK Application 
 * Programming Interface (API) Reference Manual - (For MS Windows) 
 * Version: 1.0.8 Feb. 2004 by DrRobot, Inc. and some parts from the DrRobot 
 * Java motion control demo program.
 */

/*
 * The structure for a packet to the PMS5005 is the following: STX0 = 94
 * (always) STX1 = 2 (always) RID = 1 (generally) Reserved = 0 (generally)
 * DID (Data ID) = <DID> (your choice of DID) LENGTH = <len> (Length from
 * next element to crc byte) DATA = <data> (may be more than 1 byte in
 * length) CHECKSUM = <cksum> (use crc() method to calculate on packet) ETX0 =
 * 94 (always) ETX1 = 13 (always)
 */

/**
 * Calculates a valid crc value to be used in order to check the integrity 
 * of the contents of a request packet.
 *
 * @param packet is the packetfer from which the crc value will be calculated.
 *
 * @return The crc value calculated from the given packetfer.
 */
static byte crc(Buffer* buf)
{
    byte shift_reg, sr_lsb, data_bit, v;
    int i, j, z;
    byte fb_bit;
    shift_reg = 0;               // initialize the shift register
    /* z = bufSize - 3; // Don't include crc and PMS5005.ETX (z=length-3) */
    z = buf->length;
    for (i = 0; i < z; ++i)// Don't include PMS5005.STX (i=2)
    {
	/* v = packet[i + 2] & 0x0000ffff; */
	v = packet[i] & 0x0000ffff;
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
 *     nPacket == 1 -- Send sensor data packets continuously 
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
static Buffer* motorSensorRequest(short packetNumber)
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_MOTOR_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = packetNumber;
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

    return buf;
}

/**
 * Sends a request command to the Sensing and Motion Controller (PMS5005) 
 * in order to get all the standard sensor data.
 *
 * @param packetNumber describes how many times sampled data should be 
 * sent.
 *     packetNumber == 0 -- Stop sending the sensor data packets.
 *     packetNumber == 1 -- Send sensor data packets continuously 
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
static Buffer* standardSensorRequest(short packetNumber)
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.DID.GET_MOTOR_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = packetNumber;
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

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
static Buffer* customSensorRequest(short packetNumber)
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_CUSTOM_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = packetNumber;
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

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
static Buffer* allSensorRequest(short packetNumber)
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_ALL_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = packetNumber;
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

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
static Buffer* enableMotorSensorSending()
{
    Buffer* buf = new_Buffer(9);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_MOTOR_SENSOR_DATA_DID;
	buf.data[5] = 0; // len
	buf.data[6] = crc(buf);
	buf.data[7] = PMS5005.ETX0;
	buf.data[8] = PMS5005.ETX1;
    }

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
static Buffer* enableStandardSensorSending()
{
    Buffer* buf = new_Buffer(9);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_STANDARD_SENSOR_DATA_DID;
	buf.data[5] = 0; // len
	buf.data[6] = crc(buf);
	buf.data[7] = PMS5005.ETX0;
	buf.data[8] = PMS5005.ETX1;
    }

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
static Buffer* enableCustomSensorSending()
{
    Buffer* buf = new_Buffer(9);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_CUSTOM_SENSOR_DATA_DID;
	buf.data[5] = 0; // len
	buf.data[6] = crc(buf);
	buf.data[7] = PMS5005.ETX0;
	buf.data[8] = PMS5005.ETX1;
    }

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
static Buffer* enableAllSensorSending()
{
    Buffer* buf = new_Buffer(9);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_ALL_SENSOR_DATA_DID;
	buf.data[5] = 0; // len
	buf.data[6] = crc(buf);
	buf.data[7] = PMS5005.ETX0;
	buf.data[8] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Disables batch updating of motor-related sensor packets.
 *
 * @see enableMotorSensorSending
 */
static Buffer* disableMotorSensorSending()
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_MOTOR_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = 0; //(0 & 0xff);
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Disables batch updating of standard sensor packets.
 *
 * @see enableStandardSensorSending
 */
static Buffer* disableStandardSensorSending()
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_STANDARD_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = 0; //(0 & 0xff);
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }
	
    return buf;
}

/**
 * Disables batch updating of custom sensor packets.
 *
 * @see enableCustomSensorSending
 */
static Buffer* disableCustomSensorSending()
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_CUSTOM_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = 0; //(0 & 0xff);
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Disables batch updating of all sensor packets.
 *
 * @see enableAllSensorSending
 */
static Buffer* disableAllSensorSending()
{
    Buffer* buf = new_Buffer(10);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.GET_ALL_SENSOR_DATA_DID;
	buf.data[5] = 1; // len
	buf.data[6] = 0; //(0 & 0xff);
	buf.data[7] = crc(buf);
	buf.data[8] = PMS5005.ETX0;
	buf.data[9] = PMS5005.ETX1;
    }

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
static Buffer* setMotorSensorPeriod(short timePeriod)
{
    // TODO stub
    return NULL;
}
	
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
static Buffer* setStandardSensorPeriod(short timePeriod)
{
    // TODO stub
    return NULL;
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
static Buffer* setCustomSensorPeriod(short timePeriod)
{
    // TODO stub
    return NULL;
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
static Buffer* setAllSensorPeriod(short timePeriod)
{
    // TODO stub
    return NULL;
}
	
/**
 * Returns the current distance value between the relevant ultrasonic 
 * range sensor module (DUR5200) and the object in front of it.
 *
 * @param channel 0, 1, 2, 3, 4 or 5 for Sonar #1 through #6.
 *
 * @return 4 means a distance of 0 to 4 cm to object.
 * 4 to 254 means a distance of 4 to 254 cm to object.
 * 255 means 255 cm or longer distance to object.
 *
 * Please note: By default, the sensors are indexed clockwise, starting 
 * with the left-front sensor (robot first person perspective) at Sonar #1 
 * (channel 0).
 */
static short getSensorSonar(short channel, const int standardSensorAry[])
{
    return standardSensorAry[channel + ULTRASONIC_OFFSET];
}
	
/**
 * Returns the current distance measurement value between an infrared 
 * sensor and the object in front of it.
 *
 * @param channel 0, 1, 2, 3, 4 or 5 for IR #1 through #6.
 *
 * @return <= 585 means a distance of 80 cm or longer to object.
 * 585 to 3446 means a distance of 80 to 8 cm to object.
 * >= 3446 means a distance of 0 to 8 cm to object.
 *
 * Please Note: The relationship between the return data and the distance 
 * is not linear.  Please refer to the sensor's datashee for distance-
 * voltage curve.  The data returned is the raw data of the analog to 
 * digital converter.  The output voltage of the sensor can be calculated 
 * from the following equation: sensorOutputVoltage = (ival)*3.0/4095(v)
 */
static short getSensorIrRange(short channel, const int standardSensorAry[], const int customSensorAry[])
{
    short result = -1;
		
    if (0 <= channel && channel < 1)
    {
	result = standardSensorAry[STANDARD_IR_RANGE_OFFSET + 1] << 8 | standardSensorAry[STANDARD_IR_RANGE_OFFSET];
    }
    else
    {
	result = customSensorAry[2*(channel-1) + CUSTOM_IR_RANGE_OFFSET + 1] << 8 | standardSensorAry[2*(channel-1) + CUSTOM_IR_RANGE_OFFSET];
    }
		
    return result;
}
	
/**
 * Returns the current human alarm data from the DHM5150 Human Motion 
 * Sensor Module.  Please refer to the DHM5150 hardware manual for 
 * detailed information.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @return the raw value from the analog to digital converter indicating 
 * the amplified (5x) output voltage of the sensor device.  The data range 
 * is between 0 and 4095.  When there is no human present, the module 
 * output voltage is ~1.5V and the return value is about 2047.
 *
 * Please note: To detect human presence, the application should compare 
 * the difference of two samples (to detect the change from "absence" to 
 * "presence"), and also compare the sample data to a user defined 
 * threshold (to determine whether to report an alarm or not).  The 
 * threshold determines the sensitivity of the sensor.  The higher the 
 * threshold, the lower the sensitivity will be.
 */
static short getSensorHumanAlarm(short channel, const int standardSensorAry[])
{
    int offset = 2*channel + HUMAN_ALARM_OFFSET;
    return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset];
}
	
/**
 * Returns the current human motion value from the DHM5150 Human Motion 
 * Sensor Module.  Please refer to the DHM5150 hardware manual for 
 * detailed information.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @return the un-amplified raw value of the analog to digital converter 
 * indicating the output voltage of the sensor device.  The data ranges 
 * between 0 and 4095.
 *
 * Please note: To detect human motion direction, the application should 
 * compare the difference of two samples of each sensor module's output 
 * (to detect the change from "absence" to "presence"), and then compare 
 * the sample data of the two sensor modules.  For a single source of 
 * human motion, the different patterns of the two sensor modules manifest 
 * the direction of motion.  The relationship can be obtained empirically.
 */
static short getSensorHumanMotion(short channel, const int standardSensorAry[])
{
    int offset = 2*channel + HUMAN_MOTION_OFFSET;
    return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset];
}
	
/**
 * Returns the current tilt angle value in the horizontal direction from 
 * the DTA5102 Tilting and Acceleration Sensor Module.
 *
 * @return Tilting Angle = arcsin((ival - ZeroGValue)/abs(90DegreeGValue - 
 * ZeroGValue));
 * Where 90DegreeGValue and ZeroGValue are module-specific values that can 
 * be measured by experiment:
 * 1) Place the sensor level, so that the gravity vector is perpendicular 
 *    to the measured sensor axis.
 * 2) Take the measurement and this value would be the ZeroGValue.
 * 3) Rotate the sensor so that the gravity vector is parallel with the 
 *    measured axis.
 * 4) Take the measurement and this value would be the 90DegreeGValue.
 * 5) Repeat this step for the other direction.
 * Typical value of ZeroGValue is about 2048 and abs(90DegreeGValue - 
 * ZeroGValue) is about 1250.
 */
static short getSensorTiltingX(const int standardSensorAry[])
{
    return standardSensorAry[TILTING_X_OFFSET + 1] << 8 | standardSensorAry[TILTING_X_OFFSET];
}
	
/**
 * Returns the current tilt angle value in the vertical direction from 
 * the DTA5102 Tilting and Acceleration Sensor Module.
 *
 * @return Tilting Angle = arcsin((ival - ZeroGValue)/abs(90DegreeGValue - 
 * ZeroGValue));
 * Where 90DegreeGValue and ZeroGValue are module-specific values that can 
 * be measured by experiment:
 * 1) Place the sensor level, so that the gravity vector is perpendicular 
 *    to the measured sensor axis.
 * 2) Take the measurement and this value would be the ZeroGValue.
 * 3) Rotate the sensor so that the gravity vector is parallel with the 
 *    measured axis.
 * 4) Take the measurement and this value would be the 90DegreeGValue.
 * 5) Repeat this step for the other direction.
 * Typical value of ZeroGValue is about 2048 and abs(90DegreeGValue - 
 * ZeroGValue) is about 1250.
 */
static short getSensorTiltingY(const int standardSensorAry[])
{
    return standardSensorAry[TILTING_Y_OFFSET + 1] << 8 | standardSensorAry[TILTING_Y_OFFSET];
}
	
/**
 * Returns the current air temperature values near the relevant DC motor 
 * drive modules (MDM5253).
 *
 * The current air temperature values could be used for 
 * monitoring whether the motor drivers are overheating or not.  This 
 * situation usually occurs if the motor currents are kept high (but still 
 * lower than the current limit of the motor driver module) for a 
 * significant amount of time, which may result in unfavorable inner or 
 * external system conditions and is not recommended for regular system 
 * operations.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 * 
 * @return The raw value of the analog to digital converter indicating the 
 * output voltage of the sensor.  The data range of the return value is 
 * between 0 and 4095.  The output voltage of the sensor can be calculated 
 * from the following equation: Temperature = 100 - (ival - 980)/11.6 
 * where Temperature is in degrees Celsius.
 */
static short getSensorOverheat(short channel, const int standardSensorAry[])
{
    int offset = 2*channel + OVERHEAT_SENSOR_OFFSET;
    return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset];
}
	
/**
 * Returns the current temperature value from the 
 * DA5280 Ambient Temperature Sensor Module.
 *
 * @return Temperature = (ival - 1256) / 34.8, where Temperature is in 
 * degrees Celsius.
 */
static short getSensorTemperature(const int standardSensorAry[])
{
    return standardSensorAry[TEMPERATURE_AD_OFFSET + 1] << 8 | standardSensorAry[TEMPERATURE_AD_OFFSET];
}
	
/**
 * Returns the four parts of a two-16-bit-code infrared 
 * remote control command captured by the Sensing and Motion Controller 
 * (PMS5005) through the Infrared Remote Controller Module (MIR5500).
 *
 * @param index Refers to byte 0, 1, 2, or 3.  See return value for details
 *
 * @return The infrared remote control command (4 byte code) is as follows:
 * Key Code: byte[2] byte[1] byte[0]
 * Repeat Code: byte[3]
 * Where the repeat byte would be 255 if the button is pressed continuously
 */
static short getSensorIrCode(short index, const int standardSensorAry[])
{
    return standardSensorAry[INFRARED_COMMAND_OFFSET + index];
}
	
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
static Buffer* setIrCtrlOutput(short loWord, short hiWord)
{
    // TODO Auto-generated method stub
    return null;
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
static short getSensorBatteryAd(short channel, const int standardSensorAry[])
{
    return standardSensorAry[2*channel + BATTERY_SENSOR_OFFSET + 1] << 8 | standardSensorAry[2*channel + BATTERY_SENSOR_OFFSET];
}
	
/**
 * Returns the current value of the reference voltage of the A/D converter 
 * of the controller DSP.
 *
 * @return The raw value of the analog to digital converter indicating the 
 * output voltage of the monitor.  The data range is between 0 and 4095.  
 * The following equation can be used to calculate the actual voltage 
 * values: Voltage = 6v*(ival/4095)
 */
static short getSensorRefVoltage(const int standardSensorAry[])
{
    return standardSensorAry[REFERENCE_VOLTAGE_OFFSET + 1] << 8 | standardSensorAry[REFERENCE_VOLTAGE_OFFSET];
}
	
/**
 * Returns the current value of the reference voltage of the A/D converter 
 * of the controller DSP.
 *
 * @return The raw value of the analog to digital converter indicating the 
 * output voltage of the monitor.  The data range is between 0 and 4095.  
 * The following equation can be used to calculate the actual voltage 
 * values: Voltage = 6v*(ival/4095)
 */
static short getSensorPotVoltage(const int standardSensorAry[])
{
    return standardSensorAry[POTENTIOMETER_POWER_OFFSET + 1] << 8 | standardSensorAry[POTENTIOMETER_POWER_OFFSET];
}
	
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
static short getSensorPot(short channel, const int motorSensorAry[])
{
    return motorSensorAry[2*channel + POTENTIOMETER_SENSOR_OFFSET] + 1 << 8 | motorSensorAry[2*channel + POTENTIOMETER_SENSOR_OFFSET];
}
	
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
static short getMotorCurrent(short channel, const int motorSensorAry[])
{
    return (motorSensorAry[2*channel + MOTOR_CURRENT_SENSOR_OFFSET + 1] << 8 | motorSensorAry[2*channel + MOTOR_CURRENT_SENSOR_OFFSET]) / 728.0;
}
	
/**
 * Returns +1, 0, or -1 to indicate the direction of rotation.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first 
 * person perspective).
 *
 * @return 1 to indicate positive direction, 0 to indicate no movement, 
 * and -1 to indicate negative direction.
 */
static short getEncoderDirection(short channel, const int motorSensorAry[])
{
    int offset = channel + ENCODER_DIRECTION_OFFSET;
    short result = -1;
		
    switch (channel)
    {
    case 0:
	result = motorSensorAry[offset] & 0x01;
	break;
    case 1:
	result = motorSensorAry[offset] & 0x03;
	break;
    }
		
    return result;
}
	
/**
 * Returns the current pulse counter to indicate the position of rotation.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first 
 * person perspective).
 *
 * @return Pulse counter, an short integral value to rotation with range of 
 * 0 to 32767 in cycles.
 */
static short getEncoderPulse(short channel, const int motorSensorAry[])
{
    int offset = 4*channel + ENCODER_PULSE_OFFSET;
    return motorSensorAry[offset + 1] << 8 | motorSensorAry[offset];
}
	
/**
 * Returns the rotation speed.  The unit is defined as the absolute value 
 * of the pulse change within 1 second.
 *
 * @param channel 0 for left encoder, 1 for right encoder (robot first 
 * person perspective).
 *
 * @see setDcMotorSensorUsage
 */
static short getEncoderSpeed(short channel, const int motorSensorAry[])
{
    int offset = 4*channel + MOTOR_SPEED_OFFSET;
    return motorSensorAry[offset + 1] << 8 | motorSensorAry[offset];
}
	
/**
 * Returns the sampling value of the custom analog to digital 
 * input signals.  By default, custom AD1 - AD3 are used as the inputs of 
 * power supply voltage monitors for DSP circuits, DC motors, and servo 
 * motors.  Users may change this setting by configuring the jumpers on 
 * the PMS5005.  Please refer to the PMS5005 Robot Sensing and Motion 
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
 * @see getSensorBatteryAd
 */
static short getCustomAd(short channel, const int customSensorAry[])
{
    int offset = 2*channel + CUSTOM_AD_OFFSET;
    return customSensorAry[offset + 1] << 8 | customSensorAry[offset];
}
	
/**
 * Returns a value with the lower 8 bits corresponding to the 8 channel 
 * custom digital inputs.
 *
 * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8.
 */
static short getCustomDIn(byte channel, const int customSensorAry[])
{
    // TODO Auto-generated method stub
    return 0;
}

/**
 * Sets the 8-channel custom digital outputs.
 *
 * @param ival -- only the lower 8 bits are valid and can change the 
 * corresponding outputs of the 8 channels.  The MSB of the lower byte 
 * represents channel #8 and LSB of the lower byte represents channel #1.
 */
static Buffer* setCustomDOut(byte ival)
{
    return NULL;
    // TODO Auto-generated method stub
		
}
	
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
static Buffer* setMotorPolarity(byte channel, byte polarity)
{
    Buffer* buf = new_Buffer(12);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PARAM_SET_DID;                   //DID
	buf.data[5] = 5;                           //LEN
	buf.data[6] = PMS5005.DC_SENSOR_USAGE_DID;             //Subcommand
	buf.data[7] = channel;                     //0-L | 1=R
	buf.data[8] = polarity;                    //polarity 1 | -1
	buf.data[9] = crc(buf);                 //Checksum
	buf.data[10] = PMS5005.ETX0;
	buf.data[11] = PMS5005.ETX1;
    }

    return buf;
}

/**
 * Enables the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @deprecated
 *
 * @see resumeDcMotor
 */
static Buffer* enableDcMotor(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID; //DID
	buf.data[5] = 2;                //LEN
	buf.data[6] = 1;                //1 = Enable/Resume
	buf.data[7] = channel;          //0=L | 1=R
	buf.data[8] = crc(buf);      //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Disables the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * @deprecated
 *
 * @see suspendDcMotor
 */
static Buffer* disableDcMotor(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID;         //DID
	buf.data[5] = 2;                        //LEN
	buf.data[6] = 0;                        //0 = Disable/Suspend
	buf.data[7] = channel;                  //0=L | 1=R
	buf.data[8] = crc(buf);              //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Resumes the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 */
static Buffer* resumeDcMotor(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID; //DID
	buf.data[5] = 2;                //LEN
	buf.data[6] = 1;                //resume
	buf.data[7] = channel;          //0=L | 1=R
	buf.data[8] = crc(buf);      //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Suspends the specified DC motor control channel.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 *
 * All motor control channels are initially suspended at boot-up.
 */
static Buffer* suspendDcMotor(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID; //DID
	buf.data[5] = 2;                //LEN
	buf.data[6] = 0;                //SUSPEND
	buf.data[7] = channel;          //0=L | 1=R
	buf.data[8] = crc(buf);      //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Sets up the PID control parameters of the specified DC motor channel 
 * for position control.
 *
 * @param channel 0 for left, 1 for right (robot first person perspective)
 * @param Kp proportional gain (default is 50)
 * @param Kd derivative gain (default is 5)
 * @param Ki_x100 the desired integral gain * 100.  when Ki_100 = 100, 
 * the actual integral control term is Ki = 1.  Ki_x100 has a range of 
 * 0 to 25599, where 0 means no integral control (default).
 *
 * @see setDcMotorControlMode
 */
static Buffer* setDcMotorPositionCtrlPid(byte channel, short Kp, short Kd, short Ki)
{
    Buffer* buf = new_Buffer(20);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PARAM_SET_DID;   				 //DID
	buf.data[5] = 11;                         //LEN 
	buf.data[6] = PMS5005.DC_POSITION_PID_DID;            //Subcommand
	buf.data[7] = channel;                    //0=L | 1=R
	buf.data[8] = PMS5005.KPID;                       //Proportional gain
	buf.data[9] = PMS5005.KP;        
	buf.data[10] = PMS5005.KP >> 8;
	buf.data[11] = PMS5005.KDID;                       //Derivative gain
	buf.data[12] = PMS5005.KD;        
	buf.data[13] = PMS5005.KD >> 8;
	buf.data[14] = PMS5005.KIID;                       //Integral gain
	buf.data[15] = PMS5005.KI;         
	buf.data[16] = PMS5005.KI >> 8;
	buf.data[17] = crc(buf);                //Checksum
	buf.data[18] = PMS5005.ETX0;
	buf.data[19] = PMS5005.ETX1;
    }

    return buf;
}
	
static Buffer* setDcMotorVelocityCtrlPid(byte channel, short Kp, short Kd, short Ki) 
{
    Buffer* buf = new_Buffer(20);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PARAM_SET_DID;                  //DID
	buf.data[5] = 11;                         //LEN 
	buf.data[6] = DC_VELOCITY_PID_DID;            //Subcommand
	buf.data[7] = channel;                    //0=L | 1=R
	buf.data[8] = PMS5005.KPID;                       //Proportional gain
	buf.data[9] = PMS5005.KP;
	buf.data[10] = PMS5005.KP >> 8;
	buf.data[11] = PMS5005.KDID;                       //Derivative gain
	buf.data[12] = PMS5005.KD;
	buf.data[13] = PMS5005.KD >> 8;
	buf.data[14] = PMS5005.KIID;                       //Integral gain
	buf.data[15] = PMS5005.KI;
	buf.data[16] = PMS5005.KI >> 8;
	buf.data[17] = crc(buf);                //Checksum
	buf.data[18] = PMS5005.ETX0;
	buf.data[19] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * This filtering feature is still under development. All data will be 
 * treated as raw data.
 */
static Buffer* setDcMotorSensorFilter(byte channel, short filterMethod)
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
static Buffer* setDcMotorSensorUsage(byte channel, byte sensorType)
{
    Buffer* buf = new_Buffer(12);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PARAM_SET_DID;                   //DID
	buf.data[5] = 5;                           //LEN
	buf.data[6] = PMS5005.DC_SENSOR_USAGE_DID;             //Subcommand
	buf.data[7] = channel;                     //0-5  = Single Potentiometer, 0-2  = Dual Potentiometer, 0-1  = Encoder
	buf.data[8] = sensorType;                  //0x00 = Single Potentiometer, 0x01 = Dual Potentiometer, 0x02 = Encoder
	buf.data[9] = crc(buf);                 //Checksum
	buf.data[10] = PMS5005.ETX0;
	buf.data[11] = PMS5005.ETX1;
    }

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
static Buffer* setDcMotorCtrlMode(byte channel, byte controlMode)
{
    Buffer* buf = new_Buffer(12);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PARAM_SET_DID;				//DID
	buf.data[5] = 3;						//LEN
	buf.data[6] = PMS5005.DC_CTRL_MODE_DID;			//Subcommand
	buf.data[7] = channel;				//channel 0-5
	buf.data[8] = controlMode;			//0 = open, 1 = closed position, 2 = closed velocity
	buf.data[9] = crc(buf);			//Checksum
	buf.data[10] = PMS5005.ETX0;
	buf.data[11] = PMS5005.ETX1;
    }

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
 * @param packetValue Target position value
 * @param timePeriod Executing time in milliseconds
 */
static Buffer* dcMotorPositionTimeCtrl(byte channel, short packetValue, short time)
{
    Buffer* buf = new_Buffer(14);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.POSITION_CTRL_DID;	//DID
	buf.data[5] = 5;				//LEN
	buf.data[6] = channel;			//Channel 0-5
	buf.data[7] = packetValue;		//packetValue
	buf.data[8] = packetValue >> 8;
	buf.data[9] = time;			//time
	buf.data[10] = time >> 8;
	buf.data[11] = crc(buf);		       //Checksum
	buf.data[12] = PMS5005.ETX0;
	buf.data[13] = PMS5005.ETX1;
    }

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
static Buffer* dcMotorPositionNonTimeCtrl(byte channel, short packetValue)
{
    Buffer* buf = new_Buffer(12);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.POSITION_CTRL_DID;     // DID
	buf.data[5] = 3;			        //LEN
	buf.data[6] = channel;			//channel 0-5
	buf.data[7] = packetValue;		//packetValue
	buf.data[8] = packetValue >> 8;
	buf.data[9] = crc(buf);			//Checksum
	buf.data[10] = PMS5005.ETX0;
	buf.data[11] = PMS5005.ETX1;
    }

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
static Buffer* dcMotorPwmTimeCtrl(byte channel, short packetValue, short time)
{
    Buffer* buf = new_Buffer(14);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PWM_CTRL_DID;	//DID
	buf.data[5] = 5;			//LEN
	buf.data[6] = channel;		//Channel 0-5
	buf.data[7] = packetValue;		//packetValue
	buf.data[8] = packetValue >> 8;
	buf.data[9] = time;			//time
	buf.data[10] = time >> 8;
	buf.data[11] = crc(buf);					//Checksum
	buf.data[12] = PMS5005.ETX0;
	buf.data[13] = PMS5005.ETX1;
    }

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
 * @param packetValue Target pulse width value
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
static Buffer* dcMotorPwmNonTimeCtrl(byte channel, short packetValue)
{
    Buffer* buf = new_Buffer(12);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.PWM_CTRL_DID;  //DID
	buf.data[5] = 3;			//LEN
	buf.data[6] = channel;		//Channel 0-5
	buf.data[7] = packetValue;		//packetValue
	buf.data[8] = packetValue >> 8;
	buf.data[9] = crc(buf);					//Checksum
	buf.data[10] = PMS5005.ETX0;
	buf.data[11] = PMS5005.ETX1;
    }

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
static Buffer* dcMotorPositionTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
{
    Buffer* buf = new_Buffer(23);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_POSITION_CTRL_DID; // DID
	buf.data[5] = 14;                            // LEN
	buf.data[6] = pos1;               //channel 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;               //channel 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;               //channel 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;               //channel 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;               //channel 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;
	buf.data[17] = pos6 >> 8;
	buf.data[18] = time;				//time
	buf.data[19] = time >> 8;
	buf.data[20] = crc(buf);                       //Checksum
	buf.data[21] = PMS5005.ETX0;
	buf.data[22] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Sends the position control command to all 6 DC motor control channels on 
 * the Sensing and Motion Controller (PMS5005) at the same time.  The 
 * command includes the target positions without a specified time period 
 * for execution.  The motion controller will drive the motor to reach the 
 * target position with maximum effort.
 * 
 * @param pos1 Target position for channel #1
 * @param pos2 Target position for channel #2
 * @param pos3 Target position for channel #3
 * @param pos4 Target position for channel #4
 * @param pos5 Target position for channel #5
 * @param pos6 Target position for channel #6
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
static Buffer* dcMotorPositionNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
{
    Buffer* buf = new_Buffer(21);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_POSITION_CTRL_DID;                 //DID
	buf.data[5] = 12;                                //LEN
	buf.data[6] = pos1;               //channel 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;               //channel 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;               //channel 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;               //channel 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;               //channel 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;               //channel 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = crc(buf);                       //Checksum
	buf.data[19] = PMS5005.ETX0;
	buf.data[20] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Sends the velocity control command to all 6 DC motor control channels on 
 * the Sensing and Motion Controller (PMS5005) at the same time.  The 
 * command includes the target velocities and the time period to execute 
 * the command.  The trajectory planning method for time control is linear.
 * 
 * @param pos1 Target velocity for channel #1
 * @param pos2 Target velocity for channel #2
 * @param pos3 Target velocity for channel #3
 * @param pos4 Target velocity for channel #4
 * @param pos5 Target velocity for channel #5
 * @param pos6 Target velocity for channel #6
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
static Buffer* dcMotorVelocityTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
{
    Buffer* buf = new_Buffer(23);

    if (packet != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_VELOCITY_CTRL_DID;			//DID
	buf.data[5] = 14;						//LEN
	buf.data[6] = pos1;		//MOTOR 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;		//MOTOR 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;		//MOTOR 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;		//MOTOR 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;		//MOTOR 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;		//MOTOR 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = time;		//time
	buf.data[19] = time >> 8;
	buf.data[20] = crc(buf);				//Checksum
	buf.data[21] = PMS5005.ETX0;
	buf.data[22] = PMS5005.ETX1;
    }

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
static Buffer* dcMotorVelocityNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
{
    Buffer* buf = new_Buffer(21);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_VELOCITY_CTRL_DID;			//DID
	buf.data[5] = 12;						//LEN
	buf.data[6] = pos1;		//MOTOR 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;		//MOTOR 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;		//MOTOR 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;		//MOTOR 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;		//MOTOR 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;		//MOTOR 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = crc(buf);				//Checksum
	buf.data[19] = PMS5005.ETX0;
	buf.data[20] = PMS5005.ETX1;
    }

    return buf;	
}
	
/**
 * Sends the PWM control command to all 6 DC motor control channels on the 
 * Sensing and Motion Controller (PMS5005) at the same time.  The command 
 * includes the target PWM values and the time period for execution.  The 
 * current trajectory planning method for time control is linear.
 * 
 * @param pos1 Target PWM value for channel #1
 * @param pos2 Target PWM value for channel #2
 * @param pos3 Target PWM value for channel #3
 * @param pos4 Target PWM value for channel #4
 * @param pos5 Target PWM value for channel #5
 * @param pos6 Target PWM value for channel #6
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
static Buffer* dcMotorPwmTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
{
    Buffer* buf = new_Buffer(23);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_PWM_CTRL_DID;				//DID
	buf.data[5] = 14;						//LEN
	buf.data[6] = pos1;		//MOTOR 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;		//MOTOR 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;		//MOTOR 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;		//MOTOR 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;		//MOTOR 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;		//MOTOR 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = time;		//time
	buf.data[19] = time >> 8;
	buf.data[20] = crc(buf);				//Checksum
	buf.data[21] = PMS5005.ETX0;
	buf.data[22] = PMS5005.ETX1;
    }

    return buf;
}
	
/**
 * Sends the PWM control command to all 6 DC motor control channels on the 
 * Sensing and Motion Controller (PMS5005) at the same time.  The command 
 * includes the target PWM values without a specified time period for 
 * execution.  The motion controller will adjust the pulse width right away.
 * 
 * @param pos1 Target PWM value for channel #1
 * @param pos2 Target PWM value for channel #2
 * @param pos3 Target PWM value for channel #3
 * @param pos4 Target PWM value for channel #4
 * @param pos5 Target PWM value for channel #5
 * @param pos6 Target PWM value for channel #6
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
static Buffer* dcMotorPwmNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
{
    Buffer* buf = new_Buffer(21);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_PWM_CTRL_DID;				//DID
	buf.data[5] = 12;						//LEN
	buf.data[6] = pos1;		//MOTOR 1
	buf.data[7] = pos1 >> 8;
	buf.data[8] = pos2;		//MOTOR 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;		//MOTOR 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;		//MOTOR 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;		//MOTOR 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;		//MOTOR 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = crc(buf);				//Checksum
	buf.data[19] = PMS5005.ETX0;
	buf.data[20] = PMS5005.ETX1;
    }

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
static Buffer* enableServo(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID;         //DID
	buf.data[5] = 2;                        //LEN
	buf.data[6] = 0;                        //0 = Enable
	buf.data[7] = channel;                  //6-11 SERVO
	buf.data[8] = crc(buf);              //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

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
static Buffer* disableServo(byte channel)
{
    Buffer* buf = new_Buffer(11);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.TOGGLE_DC_MOTORS_DID;         //DID
	buf.data[5] = 2;                        //LEN
	buf.data[6] = 0;                        //0 = Disable
	buf.data[7] = channel;                  //6-11 = SERVO
	buf.data[8] = crc(buf);              //Checksum
	buf.data[9] = PMS5005.ETX0;
	buf.data[10] = PMS5005.ETX1;
    }

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
static Buffer* servoTimeCtrl(byte channel, short packetValue, short time)
{
    Buffer* buf = new_Buffer(15);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.SERVO_CTRL_DID;                        //DID
	buf.data[5] = 6;                                 //LEN
	buf.data[6] = channel;                           //channel
	buf.data[7] = packetValue;           //command value low 8 bit
	buf.data[8] = packetValue >> 8;   //high 8 bit
	buf.data[9] = 6;                                 //flag
	buf.data[10] = time;               //time low 8 bit
	buf.data[11] = time >> 8;       //high 8 bit    
	buf.data[12] = crc(buf);                       //Checksum
	buf.data[13] = PMS5005.ETX0;
	buf.data[14] = PMS5005.ETX1;
    }

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
static Buffer* servoNonTimeCtrl(byte channel, short packetValue)
{
    Buffer* buf = new_Buffer(13);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.SERVO_CTRL_DID;                        //DID
	buf.data[5] = 4;                                 //LEN
	buf.data[6] = channel;                           //channel
	buf.data[7] = packetValue;           //command value low 8 bit
	buf.data[8] = packetValue >> 8;   //high 8 bit
	buf.data[9] = 6;                                 //flag
	buf.data[10] = crc(buf);                       //Checksum
	buf.data[11] = PMS5005.ETX0;
	buf.data[12] = PMS5005.ETX1;
    }

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
static Buffer* servoTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
{
    Buffer* buf = new_Buffer(23);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_SERVO_CTRL_DID;                        //DID
	buf.data[5] = 14;                                    //LEN
	buf.data[6] = pos1;				    //channel 1
	buf.data[7] = pos1 >> 8;			
	buf.data[8] = pos2;					//channel 2
	buf.data[9] = pos2 >> 8;		
	buf.data[10] = pos3;					//channel 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;					//channel 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;					//channel 5
	buf.data[15] = pos5 >> 8;
	buf.data[16] = pos6;					//channel 6
	buf.data[17] = pos6 >> 8;
	buf.data[18] = time;					//time
	buf.data[19] = time >> 8;
	buf.data[20] = crc(buf);							//Checksum
	buf.data[21] = PMS5005.ETX0;
	buf.data[22] = PMS5005.ETX1;
    }

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
 * @param pos1 Target position for channel #1 (Left Motor on X80Pro)
 * @param pos2 Target position for channel #2 (-Right Motor on X80Pro)
 * @param pos3 Target position for channel #3 (NO_CTRL on X80Pro)
 * @param pos4 Target position for channel #4 (NO_CTRL on X80Pro)
 * @param pos5 Target position for channel #5 (NO_CTRL on X80Pro)
 * @param pos6 Target position for channel #6 (NO_CTRL on X80Pro)
 * 
 * When omitting servo motors from control, please send the command value 
 * -32768 (0x8000), which implies NO_CTRL.
 * 
 * @see servoNonTimeCtrl
 */
static Buffer* servoNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
{
    Buffer* buf = new_Buffer(21);

    if (buf != NULL)
    {
	buf.data[0] = PMS5005.STX0;
	buf.data[1] = PMS5005.STX1;
	buf.data[2] = 1;
	buf.data[3] = 0;
	buf.data[4] = PMS5005.ALL_SERVO_CTRL_DID;                //DID
	buf.data[5] = 12;                            //LEN
	buf.data[6] = pos1;           //motor 1
	buf.data[7] = pos1 >> 8;             
	buf.data[8] = pos2;           //motor 2
	buf.data[9] = pos2 >> 8;
	buf.data[10] = pos3;           //motor 3
	buf.data[11] = pos3 >> 8;
	buf.data[12] = pos4;           //motor 4
	buf.data[13] = pos4 >> 8;
	buf.data[14] = pos5;           //motor 5
	buf.data[15] = pos5 >> 8;           
	buf.data[16] = pos6;           //motor 6
	buf.data[17] = pos6 >> 8;            
	buf.data[18] = crc(buf);                   //Checksum
	buf.data[19] = PMS5005.ETX0;
	buf.data[20] = PMS5005.ETX1;
    }

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
static Buffer* lcdDisplayPms(char* bmpFileName)
{
    return NULL;
    // TODO Auto-generated method stub
}

PMS5005* new_PMS5005()
{
    PMS5005->crc = crc;
    PMS5005->motorSensorRequest = motorSensorRequest;
    PMS5005->standardSensorRequest = standardSensorRequest;
    PMS5005->customSensorRequest = customSensorRequest;
    PMS5005->allSensorRequest = allSensorRequest;
    PMS5005->enableMotorSensorSending = enableMotorSensorSending;
    PMS5005->enableStandardSensorSending = enableStandardSensorSending;
    PMS5005->enableCustomSensorSending = enableCustomSensorSending;
    PMS5005->enableAllSensorSending = enableAllSensorSending;
    PMS5005->disableMotorSensorSending = disableMotorSensorSending;
    PMS5005->disableStandardSensorSending = disableStandardSensorSending;
    PMS5005->disableCustomSensorSending = disableCustomSensorSending;
    PMS5005->disableAllSensorSending = disableAllSensorSending;
    PMS5005->setMotorSensorPeriod = setMotorSensorPeriod;
    PMS5005->setStandardSensorPeriod = setStandardSensorPeriod;
    PMS5005->setCustomSensorPeriod = setCustomSensorPeriod;
    PMS5005->setAllSensorPeriod = setAllSensorPeriod;
    PMS5005->getSensorSonar = setSensorSonar;
    PMS5005->getSensorIrRange = getSensorIrRange;
    PMS5005->getSensorHumanAlarm = getSensorHumanAlarm;
    PMS5005->getSensorHumanMotion = getSensorHumanMotion;
    PMS5005->getSensorTiltingX = getSensorTiltingX;
    PMS5005->getSensorTiltingY = getSensorTiltingY;
    PMS5005->getSensorOverHeat = getSensorOverHeat;
    PMS5005->getSensorTemperature = getSensorTemperature;
    PMS5005->getSensorIrCode = getSensorIrCode;
    PMS5005->setIrCtrlOutput = setIrCtrlOutput;
    PMS5005->getSensorBatteryAd = getSensorBatteryAd;
    PMS5005->getSensorRefVoltage = getSensorRefVoltage;
    PMS5005->getSensorPotVoltage = getSensorPotVoltage;
    PMS5005->getSensorPot = getSensorPot;
    PMS5005->getMotorCurrent = getMotorCurrent;
    PMS5005->getEncoderDirection = getEncoderDirection;
    PMS5005->getEncoderPulse = getEncoderPulse;
    PMS5005->getEncoderSpeed = getEncoderSpeed;
    PMS5005->getCustomAd = getCustomAd;
    PMS5005->getCustomDIn = getCustomDIn;
    PMS5005->getCustomDOut = getCustomDOut;
    PMS5005->setMotorPolarity = setMotorPolarity;
    PMS5005->enableDcMotor = enableDcMotor;
    PMS5005->disableDcMotor = disableDcMotor;
    PMS5005->resumeDcMotor = resumeDcMotor;
    PMS5005->suspendDcMotor = suspendDcMotor;
    PMS5005->setDcMotorPositionCtrlPid = setDcMotorPositionCtrlPid;
    PMS5005->setDcMotorVelocityPid = setDcMotorVelocityPid;
    PMS5005->setDcMotorSensorFilter = setDcMotorSensorFilter;
    PMS5005->setDcMotorSensorUsage = setDcMotorSensorUsage;
    PMS5005->setDcMotorCtrlMode = setDcMotorCtrlMode;
    PMS5005->dcMotorPositionTimeCtrl = dcMotorPositionTimeCtrl;
    PMS5005->dcMotorPositionNonTimeCtrl = dcMotorPositionNonTimeCtrl;
    PMS5005->dcMotorPwmTimeCtrl = dcMotorPwmTimeCtrl;
    PMS5005->dcMotorPwmNonTimeCtrl = dcMotorPwmNonTimeCtrl;
    PMS5005->dcMotorPositionTimeCtrlAll = dcMotorPositionTimeCtrlAll;
    PMS5005->dcMotorPositionNonTimeCtrlAll = dcMotorPositionNonTimeCtrlAll;
    PMS5005->dcMotorVelocityTimeCtrlAll = dcMotorVelocityTimeCtrlAll;
    PMS5005->dcMotorVelocityNonTimeCtrlAll = dcMotorVelocityNonTimeCtrlAll;
    PMS5005->dcMotorPwmTimeCtrlAll = dcMotorPwmTimeCtrlAll;
    PMS5005->dcMotorPwmNonTimeCtrlAll = dcMotorPwmNonTimeCtrlAll;
    PMS5005->dcMotorPositionTimeCtrlBoth = dcMotorPositionTimeCtrlBoth;
    PMS5005->dcMotorPositionNonTimeCtrlBoth = dcMotorPositionNonTimeCtrlBoth;
    PMS5005->dcMotorVelocityTimeCtrlBoth = dcMotorVelocityTimeCtrlBoth;
    PMS5005->dcMotorVelocityNonTimeCtrlBoth = dcMotorVelocityNonTimeCtrlBoth;
    PMS5005->dcMotorPwmTimeCtrlBoth = dcMotorPwmTimeCtrlBoth;
    PMS5005->dcMotorPwmNonTimeCtrlBoth = dcMotorPwmNonTimeCtrlBoth;
    PMS5005->enableServo = enableServo;
    PMS5005->disableServo = disableServo;
    PMS5005->servoTimeCtrl = servoTimeCtrl;
    PMS5005->servoNonTimeCtrl = servoNonTimeCtrl;
    PMS5005->servoTimeCtrlAll = servoTimeCtrlAll;
    PMS5005->servoNonTimeCtrlAll = servoNonTimeCtrlAll;
    PMS5005->servoTimeCtrlBoth = servoTimeCtrlBoth;
    PMS5005->servoNonTimeCtrlBoth = servoNonTimeCtrlBoth;
    PMS5005->lcdDisplayPms = lcdDisplayPms;

    reutrn PMS5005;
}
