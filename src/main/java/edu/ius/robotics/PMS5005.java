package edu.ius.robotics;

public class PMS5005
{
	/* 
	 * Java implmentation written by Jesse Riddle and Colton Jenkins, with 
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
	 * length) CHECKSUM = <cksum> (use crc() method to calculate on cmd) ETX0 =
	 * 94 (always) ETX1 = 13 (always)
	 */
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 94;
    public static final byte STX1 = 2;
    public static final byte ETX0 = 94;
    public static final byte ETX1 = 13;
	
	/* Data ID (DID) descriptor listing */
	public static final byte POSITION_CTRL = 3;
	public static final byte ALL_POSITION_CTRL = 4;
	public static final byte PWM_CTRL = 5;
	public static final byte ALL_PWM_CTRL = 6;
	public static final byte PARAM_SET = 7;
		//Subcommands under PARAM_SET
		public static final byte DC_POSITION_PID = 7;     // positon PID Control
		public static final byte DC_VELOCITY_PID = 8;     // velocity PID Control
		public static final byte DC_SENSOR_USAGE = 13;
		public static final byte DC_CTRL_MODE = 14;
		
	public static final byte POWER_CTRL = 22;
	public static final byte LCD_CTRL = 23;
	public static final byte VELOCITY_CTRL = 26;
	public static final byte ALL_VELOCITY_CTRL = 27;
	public static final byte SERVO_CTRL = 28;
	public static final byte ALL_SERVO_CTRL = 29;
	public static final byte TOGGLE_DC_MOTORS = 30;
	public static final byte CONSTELLATION_CTRL = 80;
	public static final byte GET_MOTOR_SENSOR_DATA = 123;
	public static final byte GET_CUSTOM_SENSOR_DATA = 124;
	public static final byte GET_STANDARD_SENSOR_DATA = 125;
	public static final byte GET_ALL_SENSOR_DATA = 127;
	// to use as ubyte: (byte)(SETUP_COM & 0xff)
	public static final short SETUP_COM = 255;
	/* End Data ID (DID) descriptor listing */
	
	

	public static final byte PWM_CTRL_MODE = 0;
	public static final byte POSITION_CTRL_MODE = 1;
	public static final byte VELOCITY_CTRL_MODE = 2;
	
	public static final byte KpId = 1; // progressive id
	public static final byte KdId = 2; // derivative id
	public static final byte KiId = 3; // integral id
	
	public static final short NON_CTRL_CMD = (short) 0xffff; // no ctrl command
	public static final short NO_CTRL = (short) 0x8000;
	
	public static final int ULTRASONIC_OFFSET = 0;
	public static final int ENCODER_PULSE_OFFSET = 24; 
	public static final int ENCODER_SPEED_OFFSET = 32;
	public static final int IR_RANGE_OFFSET = 24;
	public static final int HUMAN_ALARM_OFFSET = 6;
	public static final int HUMAN_MOTION_OFFSET = 8;
	public static final int TILTING_X_OFFSET = 14;
	public static final int TILTING_Y_OFFSET = 16;
	public static final int ENCODER_DIRECTION_OFFSET = 32;
	public static final int MOTOR_SPEED_OFFSET = 26;
	public static final int CUSTOM_AD_OFFSET = 0;
	public static final int TEMPERATURE_AD_OFFSET = 22;
	public static final int OVERHEAT_SENSOR_OFFSET = 18;
	
    /**
     * Calculates a valid crc value to be used in order to check the integrity 
     * of the contents of a request packet.
     *
     * @param buf is the buffer from which the crc value will be calculated.
     *
     * @return The crc value calculated from the given buffer.
     */
	public static byte crc(byte[] buf)
	{
		byte shift_reg, sr_lsb, data_bit, v;
		byte fb_bit;
		short z;
		shift_reg = 0; // initialize the shift register
		z = (short) (buf.length - 3);// Don't include crc and ETX (z=length-3)
		for (short i = 2; i < z; ++i)// Don't include STX (i=2)
		{
			v = (byte) (buf[i]); 
			// for each bit
			
			for (short j = 0; j < 8; ++j)
			{
				// isolate least sign bit
				data_bit = (byte) ((v & 0x01) & 0xff);
				sr_lsb = (byte) ((shift_reg & 0x01) & 0xff);
				// calculate the feed back bit
				fb_bit = (byte) (((data_bit ^ sr_lsb) & 0x01) & 0xff);
				shift_reg = (byte) ((shift_reg & 0xff) >>> 1);
				
				if (fb_bit == 1)
				{
					shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xff);
				}
				
				v = (byte) ((v & 0xff) >>> 1);
			}
		}
		
		return shift_reg;
	}
	
    /**
     * Sends a request command to the Sensing and Motion Controller (PMS5005) 
     * in order to get the sensor data related to motor control.
     *
     * @param packetNumber describes how many times sampled data should be sent
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
     * See Also: setMotorSensorPeriod
     */
	public static byte[] motorSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
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
	public static byte[] standardSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
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
	public static byte[] customSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
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
	public static byte[] allSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
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
	public static byte[] enableMotorSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = crc(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
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
	public static byte[] enableStandardSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_STANDARD_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = crc(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
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
	public static byte[] enableCustomSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = crc(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
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
	public static byte[] enableAllSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = crc(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
	}
	
    /**
     * Disables batch updating of motor-related sensor packets.
     *
     * @see enableMotorSensorSending
     */
	public static byte[] disableMotorSensorSending()
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; //(byte) (0 & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
    /**
     * Disables batch updating of standard sensor packets.
     *
     * @see enableStandardSensorSending
     */
	public static byte[] disableStandardSensorSending()
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_STANDARD_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; //(byte) (0 & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
    /**
     * Disables batch updating of custom sensor packets.
     *
     * @see enableCustomSensorSending
     */
	public static byte[] disableCustomSensorSending()
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; //(byte) (0 & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
    /**
     * Disables batch updating of all sensor packets.
     *
     * @see enableAllSensorSending
     */
	public static byte[] disableAllSensorSending()
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = 1;
		cmd[3] = 0;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; //(byte) (0 & 0xff);
		cmd[7] = crc(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
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
	public static byte[] setMotorSensorPeriod(short timePeriod)
	{
		// TODO stub
		return null;
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
	public static byte[] setStandardSensorPeriod(short timePeriod)
	{
		// TODO stub
		return null;
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
	public static byte[] setCustomSensorPeriod(short timePeriod)
	{
		// TODO stub
		return null;
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
	public static byte[] setAllSensorPeriod(short timePeriod)
	{
		// TODO stub
		return null;
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
	public static short getSensorSonar(short channel, int[] standardSensorAry)
	{
		return (byte) (standardSensorAry[channel + ULTRASONIC_OFFSET] & 0xff);
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
	public static short getSensorIrRange(short channel, int[] standardSensorAry)
	{
		// TODO channel
		return (short) (((standardSensorAry[IR_RANGE_OFFSET + 1] & 0xff) << 8) | (standardSensorAry[IR_RANGE_OFFSET] & 0xff));
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
	public static short getSensorHumanAlarm(short channel, int[] standardSensorAry)
	{
		int offset = 2*channel + HUMAN_ALARM_OFFSET;
		return (short) (((standardSensorAry[offset + 1] & 0xff) << 8) | (standardSensorAry[offset] & 0xff));
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
	public static short getSensorHumanMotion(short channel, int[] standardSensorAry)
	{
		int offset = 2*channel + HUMAN_MOTION_OFFSET;
		return (short) (((standardSensorAry[offset + 1] & 0xff) << 8) | (standardSensorAry[offset] & 0xff));
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
	public static short getSensorTiltingX(int[] standardSensorAry)
	{
		return (short) (((standardSensorAry[TILTING_X_OFFSET + 1] & 0xff) << 8) | (standardSensorAry[TILTING_X_OFFSET] & 0xff));
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
	public static short getSensorTiltingY(int[] standardSensorAry)
	{
		return (short) (((standardSensorAry[TILTING_X_OFFSET + 1] & 0xff) << 8) | (standardSensorAry[TILTING_X_OFFSET] & 0xff));
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
	public static short getSensorOverheat(short channel, int[] standardSensorAry)
	{
		int offset = 2*channel + OVERHEAT_SENSOR_OFFSET;
		return (short) (((standardSensorAry[offset + 1] & 0xff) << 8) | (standardSensorAry[offset] & 0xff));
	}
	
    /**
     * Returns the current temperature value from the 
     * DA5280 Ambient Temperature Sensor Module.
     *
     * @return Temperature = (ival - 1256) / 34.8, where Temperature is in 
     * degrees Celsius.
     */
	public static short getSensorTemperature(int[] standardSensorAry)
	{
		return (short) (((standardSensorAry[TEMPERATURE_AD_OFFSET + 1] & 0xff) << 8) | (standardSensorAry[TEMPERATURE_AD_OFFSET] & 0xff));
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
	public static short getSensorIrCode(short index)
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static byte[] setInfraredControlOutput(short lowWord, short highWord)
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
	public static short getSensorBatteryAd(short channel)
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static short getSensorRefVoltage()
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static short getSensorPotVoltage()
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static short getSensorPot(short channel)
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static short getMotorCurrent(short channel)
	{
		// TODO Auto-generated method stub
		return 0;
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
	public static short getEncoderDirection(short channel, int[] motorSensorAry)
	{
		int offset = channel + ENCODER_DIRECTION_OFFSET;
		short result = -1;
		
		switch (channel)
		{
		case 0:
			result = (short) (motorSensorAry[offset] & 0x01);
			break;
		case 1:
			result = (short) (motorSensorAry[offset] & 0x03);
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
	public static short getEncoderPulse(short channel, int[] motorSensorAry)
	{
		int offset = 4*channel + ENCODER_PULSE_OFFSET;
		return (short) (((motorSensorAry[offset + 1] & 0xff) << 8) | (motorSensorAry[offset] & 0xff));
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
	public static short getEncoderSpeed(short channel, int[] motorSensorAry)
	{
		int offset = 4*channel + MOTOR_SPEED_OFFSET;
		return (short) (((motorSensorAry[offset + 1] & 0xff) << 8) | (motorSensorAry[offset] & 0xff));
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
	public static short getCustomAd(short channel, int[] customSensorAry)
	{
		int offset = 2*channel + CUSTOM_AD_OFFSET;
		return (short) (((customSensorAry[offset+1] & 0xff) << 8) | (customSensorAry[offset] & 0xff));
	}
	
    /**
     * Returns a value with the lower 8 bits corresponding to the 8 channel 
     * custom digital inputs.
     *
     * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8.
     */
	public static short getCustomDIn(short channel)
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
	public static byte[] setCustomDOut(short ival)
	{
		return null;
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
	public static byte[] setMotorPolarity(byte channel, byte polarity)
	{
		byte[] packet = new byte[6];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4] = PARAM_SET;                   //DID
		packet[5] = 5;                           //LEN
		packet[6] = DC_SENSOR_USAGE;             //Subcommand
		packet[7] = channel;                     //0-L | 1=R
		packet[8] = polarity;                    //polarity 1 | -1
		packet[9] = crc(packet);                 //Checksum
		
		packet[4]  = ETX0;  packet[5]  = ETX0;
		return packet;
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
	public static byte[] enableDcMotor(byte channel)
	{
		byte[] packet = new byte[11];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = TOGGLE_DC_MOTORS; //DID
		packet[5]  = 2;                //LEN
		packet[6]  = 1;                //1 = Enable/Resume
		packet[7]  = channel;          //0=L | 1=R
		packet[8]  = crc(packet);      //Checksum
		
		packet[9]  = ETX0;	packet[10] = ETX1;
		return packet;
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
	public static byte[] disableDcMotor(byte channel)
	{
		byte[] packet = new byte[10];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1; 		packet[3] = 0;
		
		packet[4] = TOGGLE_DC_MOTORS;         //DID
		packet[5] = 2;                        //LEN
		packet[6] = 0;                        // 0 = Disable/Suspend
		packet[7] = channel;                  //0=L | 1=R
		packet[7] = crc(packet);              //Checksum
		
		packet[8] = ETX0;	packet[9] = ETX1;
		return packet;
	}
	
	/**
     * Resumes the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     */
	public static byte[] resumeDcMotor(byte channel)
	{
		byte[] packet = new byte[11];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = TOGGLE_DC_MOTORS; //DID
		packet[5]  = 2;                //LEN
		packet[6]  = 1;                //resume
		packet[7]  = channel;          //0=L | 1=R
		packet[8]  = crc(packet);      //Checksum
		
		packet[9]  = ETX0;	packet[10] = ETX1;
		return packet;
	}
	
	/**
     * Suspends the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public static byte[] suspendDcMotor(byte channel)
	{
		byte[] packet = new byte[11];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = TOGGLE_DC_MOTORS; //DID
		packet[5]  = 2;                //LEN
		packet[6]  = 0;                //SUSPEND
		packet[7]  = channel;          //0=L | 1=R
		packet[8]  = crc(packet);      //Checksum
		
		packet[9]  = ETX0;	packet[10] = ETX1;
		return packet;
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
	public static byte[] setDcMotorPositionControlPid(byte channel, short Kp, short Kd, short Ki)
	{
		byte[] packet = new byte[20];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = PARAM_SET;   				 //DID
		packet[5]  = 11;                         //LEN 
		packet[6]  = DC_POSITION_PID;            //Subcommand
		packet[7]  = channel;                    //0=L | 1=R
		packet[8]  = KpId;                       //Proportional gain
		packet[9]  = (byte) (Kp & 0xff);        
	    packet[10] = (byte) ((Kp >>> 8) & 0xff);
        packet[11] = KdId;                       //Derivative gain
        packet[12] = (byte) (Kd & 0xff);        
        packet[13] = (byte) ((Kd >>> 8) & 0xff);
        packet[14] = KiId;                       //Integral gain
        packet[15] = (byte) (Ki & 0xff);         
        packet[16] = (byte) ((Ki >>> 8) & 0xff);
        packet[17] = crc(packet);                //Checksum

	    packet[18] = ETX0; packet[19] = ETX1;
	    return packet;
	}
	
    public static byte[] setDcMotorVelocityControlPID(byte channel, int Kp, int Kd, int Ki) 
    {
    	byte[] packet = new byte[20];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = PARAM_SET;                  //DID
		packet[5]  = 11;                         //LEN 
		packet[6]  = DC_VELOCITY_PID;            //Subcommand
		packet[7]  = channel;                    //0=L | 1=R
		packet[8]  = KpId;                       //Proportional gain
		packet[9]  = (byte) (Kp & 0xff);         
	    packet[10] = (byte) ((Kp >>> 8) & 0xff);
        packet[11] = KdId;                       //Derivative gain
        packet[12] = (byte) (Kd & 0xff);         
        packet[13] = (byte) ((Kd >>> 8) & 0xff);
        packet[14] = KiId;                       //Integral gain
        packet[15] = (byte) (Ki & 0xff);         
        packet[16] = (byte) ((Ki >>> 8) & 0xff);
        packet[17] = crc(packet);                //Checksum

	    packet[18] = ETX0; packet[19] = ETX1;
	    return packet;
    }
	
	/**
     * This filtering feature is still under development. All data will be 
     * treated as raw data.
     */
	public static byte[] setDcMotorSensorFilter(short channel, short filterMethod)
	{
		// TODO Auto-generated method stub
		byte[] packet = new byte[6];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = ETX0;  packet[5]  = ETX0;
		return packet;
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
	public static byte[] setDcMotorSensorUsage(byte channel, byte sensorType)
	{
		byte[] packet = new byte[6];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4] = PARAM_SET;                   //DID
		packet[5] = 5;                           //LEN
		packet[6] = DC_SENSOR_USAGE;             //Subcommand
		packet[7] = channel;                     //0-5  = Single Potentiometer, 0-2  = Dual Potentiometer, 0-1  = Encoder
		packet[8] = sensorType;                  //0x00 = Single Potentiometer, 0x01 = Dual Potentiometer, 0x02 = Encoder
		packet[9] = crc(packet);                 //Checksum
		
		packet[4]  = ETX0;  packet[5]  = ETX0;
		return packet;
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
	public static byte[] setDcMotorControlMode(byte channel, byte controlMode)
	{
		byte[] packet = new byte[6];
		packet[0]  = STX0;	packet[1]  = STX1;
		packet[2]  = 1;		packet[3]  = 0;
		
		packet[4]  = PARAM_SET;				//DID
		packet[5]  = 3;						//LEN
		packet[6]  = DC_CTRL_MODE;			//Subcommand
		packet[7]  = channel;				//channel 0-5
		packet[8]  = controlMode;			//0 = open, 1 = closed position, 2 = closed velocity
		packet[9]  = crc(packet);			//Checksum
		
		packet[4]  = ETX0;  packet[5]  = ETX0;
		return packet;
	}
	
	/**
     * Sends the position control command to the specified motion control 
     * channel on the Sensing and Motion Controller (PMS5005).  
     * The command includes the target position and the target time 
     * period to execute the command.  The current trajectory planning method 
     * with time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target position value
     * @param timePeriod Executing time in milliseconds
     */
	public static byte[] dcMotorPositionTimeCtrl(byte channel, short cmdValue, short time)
	{
		byte[] packet = new byte[14];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = POSITION_CTRL;					//DID
		packet[5]  = 5;								//LEN
		packet[6]  = channel;						//Channel 0-5
		packet[7]  = (byte)(cmdValue & 0xFF);		//cmdValue
		packet[8]  = (byte)((cmdValue >>> 8) & 0xFF);
		packet[9]  = (byte)(time & 0xFF);			//time
		packet[10] = (byte)((time >>> 8) & 0xFF);
		packet[11] = crc(packet);					//Checksum
		
		packet[12] = ETX0;	packet[13] = ETX1;
		return packet;
	}
	
	/**
     * Sends the position control command to the specified motion control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position but no time period specified to execute 
     * the command.  The motion controller will drive the motor to the target 
     * position at the maximum speed.
     *
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target position value
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
	public static byte[] dcMotorPositionNonTimeCtrl(byte channel, short cmdValue)
	{
		byte[] packet = new byte[12];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4] = POSITION_CTRL;					//DID
		packet[5] = 3;								//LEN
		packet[6] = channel;						//channel 0-5
		packet[7] = (byte)(cmdValue & 0xFF);		//cmdValue
		packet[8] = (byte)((cmdValue >>> 8) & 0xFF);
		packet[9] = crc(packet);					//Checksum
		
		packet[10] = ETX0;	packet[11] = ETX1;
		return packet;
	}
	
	/**
     * Sends the PWM control command to the specified motion control channel on 
     * the Sensing and Motion Controller (PMS5005).  The command includes the 
     * target pulse width value and the time period to execute the command. 
     * The current trajectory planning method for time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width value
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
	public static byte[] dcMotorPwmTimeCtrl(byte channel, short cmdValue, short time)
	{
		byte[] packet = new byte[14];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = PWM_CTRL;						//DID
		packet[5]  = 5;								//LEN
		packet[6]  = channel;						//Channel 0-5
		packet[7]  = (byte)(cmdValue & 0xFF);		//cmdValue
		packet[8]  = (byte)((cmdValue >>> 8) & 0xFF);
		packet[9]  = (byte)(time & 0xFF);			//time
		packet[10] = (byte)((time >>> 8) & 0xFF);
		packet[11] = crc(packet);					//Checksum
		
		packet[12] = ETX0;	packet[13] = ETX1;
		return packet;
	}
	
	/**
     * Sends the PWM control command to the specified motion control channel on 
     * the Sensing and Motion Controller (PMS5005).  The command includes the 
     * target pulse width value without an execution time period specified.  
     * The motion controller will set the PWM output of this channel to the 
     * target value immediately.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width value
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
	public static byte[] dcMotorPwmNonTimeCtrl(byte channel, short cmdValue)
	{
		byte[] packet = new byte[12];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4] = PWM_CTRL;						//DID
		packet[5] = 3;								//LEN
		packet[6] = channel;						//Channel 0-5
		packet[7] = (byte)(cmdValue & 0xFF);		//cmdValue
		packet[8] = (byte)((cmdValue >>> 8) & 0xFF);
		packet[9] = crc(packet);					//Checksum
		
		packet[10] = ETX0;	packet[11] = ETX1;
		return packet;
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
	public static byte[] dcMotorPositionTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
	{
		byte[] packet = new byte[23];
		packet[0] = STX0;  packet[1] = STX1;
		packet[2] = 1;   packet[3] = 0;
		
		packet[4]  = ALL_POSITION_CTRL;                 //DID
		packet[5]  = 14;                                //LEN
		packet[6]  = (byte)(pos1 & 0xff);               //channel 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xff);
		packet[8]  = (byte)(pos2 & 0xff);               //channel 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xff);
		packet[10] = (byte)(pos3 & 0xff);               //channel 3
		packet[11] = (byte)((pos3 >>> 8) & 0xff);
		packet[12] = (byte)(pos4 & 0xff);               //channel 4
		packet[13] = (byte)((pos4 >>> 8) & 0xff);
		packet[14] = (byte)(pos5 & 0xff);               //channel 5
		packet[15] = (byte)((pos5 >>> 8) & 0xff);
		packet[16] = (byte)(pos6 & 0xff);               //channel 6
		packet[17] = (byte)((pos6 >>> 8) & 0xff);
		packet[18] = (byte)(time & 0xff);				//time
		packet[19] = (byte)((time >>> 8) & 0x0ff);
		packet[20] = crc(packet);                       //Checksum

		packet[21] = ETX0; packet[22] = ETX1;
		return packet;
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
	public static byte[] dcMotorPositionNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
	{
		byte[] packet = new byte[21];
		packet[0] = STX0;  packet[1] = STX1;
		packet[2] = 1;   packet[3] = 0;
		
		packet[4]  = ALL_POSITION_CTRL;                 //DID
		packet[5]  = 12;                                //LEN
		packet[6]  = (byte)(pos1 & 0xff);               //channel 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xff);
		packet[8]  = (byte)(pos2 & 0xff);               //channel 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xff);
		packet[10] = (byte)(pos3 & 0xff);               //channel 3
		packet[11] = (byte)((pos3 >>> 8) & 0xff);
		packet[12] = (byte)(pos4 & 0xff);               //channel 4
		packet[13] = (byte)((pos4 >>> 8) & 0xff);
		packet[14] = (byte)(pos5 & 0xff);               //channel 5
		packet[15] = (byte)((pos5 >>> 8) & 0xff);
		packet[16] = (byte)(pos6 & 0xff);               //channel 6
		packet[17] = (byte)((pos6 >>> 8) & 0xff);
		packet[18] = crc(packet);                       //Checksum

		packet[19] = ETX0; packet[20] = ETX1;
		return packet;
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
	public static byte[] dcMotorVelocityTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
	{
		byte[] packet = new byte[23];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = ALL_VELOCITY_CTRL;			//DID
		packet[5]  = 14;						//LEN
		packet[6]  = (byte)(pos1 & 0xFF);		//MOTOR 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xFF);
		packet[8]  = (byte)(pos2 & 0xFF);		//MOTOR 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xFF);
		packet[10] = (byte)(pos3 & 0xFF);		//MOTOR 3
		packet[11] = (byte)((pos3 >>> 8) & 0xFF);
		packet[12] = (byte)(pos4 & 0xFF);		//MOTOR 4
		packet[13] = (byte)((pos4 >>> 8) & 0xFF);
		packet[14] = (byte)(pos5 & 0xFF);		//MOTOR 5
		packet[15] = (byte)((pos5 >>> 8) & 0xFF);
		packet[16] = (byte)(pos6 & 0xFF);		//MOTOR 6
		packet[17] = (byte)((pos6 >>> 8) & 0xFF);
		packet[18] = (byte)(time & 0xFF);		//time
		packet[19] = (byte)((time >>> 8) & 0xFF);
		packet[20] = crc(packet);				//Checksum
		
		packet[21] = ETX0;  packet[22] = ETX1;
		return packet;
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
	public static byte[] dcMotorVelocityNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
	{
		byte[] packet = new byte[21];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = ALL_VELOCITY_CTRL;			//DID
		packet[5]  = 12;						//LEN
		packet[6]  = (byte)(pos1 & 0xFF);		//MOTOR 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xFF);
		packet[8]  = (byte)(pos2 & 0xFF);		//MOTOR 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xFF);
		packet[10] = (byte)(pos3 & 0xFF);		//MOTOR 3
		packet[11] = (byte)((pos3 >>> 8) & 0xFF);
		packet[12] = (byte)(pos4 & 0xFF);		//MOTOR 4
		packet[13] = (byte)((pos4 >>> 8) & 0xFF);
		packet[14] = (byte)(pos5 & 0xFF);		//MOTOR 5
		packet[15] = (byte)((pos5 >>> 8) & 0xFF);
		packet[16] = (byte)(pos6 & 0xFF);		//MOTOR 6
		packet[17] = (byte)((pos6 >>> 8) & 0xFF);
		packet[18] = crc(packet);				//Checksum
		
		packet[19] = ETX0;  packet[20] = ETX1;
		return packet;	
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
	public static byte[] dcMotorPwmTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
	{
		byte[] packet = new byte[23];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = ALL_PWM_CTRL;				//DID
		packet[5]  = 14;						//LEN
		packet[6]  = (byte)(pos1 & 0xFF);		//MOTOR 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xFF);
		packet[8]  = (byte)(pos2 & 0xFF);		//MOTOR 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xFF);
		packet[10] = (byte)(pos3 & 0xFF);		//MOTOR 3
		packet[11] = (byte)((pos3 >>> 8) & 0xFF);
		packet[12] = (byte)(pos4 & 0xFF);		//MOTOR 4
		packet[13] = (byte)((pos4 >>> 8) & 0xFF);
		packet[14] = (byte)(pos5 & 0xFF);		//MOTOR 5
		packet[15] = (byte)((pos5 >>> 8) & 0xFF);
		packet[16] = (byte)(pos6 & 0xFF);		//MOTOR 6
		packet[17] = (byte)((pos6 >>> 8) & 0xFF);
		packet[18] = (byte)(time & 0xFF);		//time
		packet[19] = (byte)((time >>> 8) & 0xFF);
		packet[20] = crc(packet);				//Checksum
		
		packet[21] = ETX0;  packet[22] = ETX1;
		return packet;
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
	public static byte[] dcMotorPwmNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
	{
		byte[] packet = new byte[21];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = ALL_PWM_CTRL;				//DID
		packet[5]  = 12;						//LEN
		packet[6]  = (byte)(pos1 & 0xFF);		//MOTOR 1
		packet[7]  = (byte)((pos1 >>> 8) & 0xFF);
		packet[8]  = (byte)(pos2 & 0xFF);		//MOTOR 2
		packet[9]  = (byte)((pos2 >>> 8) & 0xFF);
		packet[10] = (byte)(pos3 & 0xFF);		//MOTOR 3
		packet[11] = (byte)((pos3 >>> 8) & 0xFF);
		packet[12] = (byte)(pos4 & 0xFF);		//MOTOR 4
		packet[13] = (byte)((pos4 >>> 8) & 0xFF);
		packet[14] = (byte)(pos5 & 0xFF);		//MOTOR 5
		packet[15] = (byte)((pos5 >>> 8) & 0xFF);
		packet[16] = (byte)(pos6 & 0xFF);		//MOTOR 6
		packet[17] = (byte)((pos6 >>> 8) & 0xFF);
		packet[18] = crc(packet);				//Checksum
		
		packet[19] = ETX0;  packet[20] = ETX1;
		return packet;
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
	public static byte[] enableServo(byte channel)
	{
		byte[] packet = new byte[10];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1; 		packet[3] = 0;
		
		packet[4] = TOGGLE_DC_MOTORS;         //DID
		packet[5] = 2;                        //LEN
		packet[6] = 0;                        //0 = Enable
		packet[7] = channel;                  //6-11 SERVO
		packet[7] = crc(packet);              //Checksum
		
		packet[8] = ETX0;	packet[9] = ETX1;
		return packet;
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
	public static byte[] disableServo(byte channel)
	{
		byte[] packet = new byte[10];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1; 		packet[3] = 0;
		
		packet[4] = TOGGLE_DC_MOTORS;         //DID
		packet[5] = 2;                        //LEN
		packet[6] = 0;                        //0 = Disable
		packet[7] = channel;                  //6-11 = SERVO
		packet[7] = crc(packet);              //Checksum
		
		packet[8] = ETX0;	packet[9] = ETX1;
		return packet;
	}
	
	/**
     * Sends the position control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command and the time period to execute the 
     * command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target Pulse Width (in milliseconds) * 2250
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
	public static byte[] servoTimeCtrl(byte channel, short cmdValue, short time)
	{
		byte[] packet = new byte[15];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = SERVO_CTRL;                        //DID
		packet[5]  = 6;                                 //LEN
		packet[6]  = channel;                           //channel
		packet[7]  = (byte)(cmdValue & 0xff);           //command value low 8 bit
        packet[8]  = (byte)((cmdValue >>> 8) & 0xff);   //high 8 bit
        packet[9]  = 6;                                 //flag
        packet[10] = (byte)(time & 0xff);               //time low 8 bit
        packet[11] = (byte)((time >>> 8) & 0xff);       //high 8 bit    
        packet[12] = crc(packet);                       //Checksum
        
        packet[13] = ETX0;	packet[14] = ETX1;
        return packet;
	}
	
	/**
     * Sends the position control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command without a specific time period for 
     * execution.  The motion controller will send the desired pulse width to 
     * the servo motor right away.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width (ms) * 2250
     * 
     * @see servoTimeCtrl
     */
	public static byte[] servoNonTimeCtrl(byte channel, short cmdValue)
	{
		byte[] packet = new byte[13];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1;		packet[3] = 0;
		
		packet[4]  = SERVO_CTRL;                        //DID
		packet[5]  = 4;                                 //LEN
		packet[6]  = channel;                           //channel
		packet[7]  = (byte)(cmdValue & 0xff);           //command value low 8 bit
        packet[8]  = (byte)((cmdValue >>> 8) & 0xff);   //high 8 bit
        packet[9]  = 6;                                 //flag
        packet[10] = crc(packet);                       //Checksum
        
        packet[11] = ETX0;	packet[12] = ETX1;
        return packet;
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
	public static byte[] servoTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short time)
	{
		byte[] packet = new byte[23];
		packet[0] = STX0;	packet[1] = STX1;
		packet[2] = 1; 	packet[3] = 0;
		
		packet[4]  = ALL_SERVO_CTRL;                        //DID
		packet[5]  = 14;                                    //LEN
		packet[6]  = (byte) (pos1 & 0xff);				    //channel 1
		packet[7]  = (byte) ((pos1 >>> 8) & 0xff);			
		packet[8]  = (byte) (pos2 & 0xff);					//channel 2
		packet[9]  = (byte) ((pos2 >>> 8) & 0xff);		
		packet[10] = (byte) (pos3 & 0xff);					//channel 3
		packet[11] = (byte) ((pos3 >>> 8) & 0xff);
		packet[12] = (byte) (pos4 & 0xff);					//channel 4
		packet[13] = (byte) ((pos4 >>> 8) & 0xff);
		packet[14] = (byte) (pos5 & 0xff);					//channel 5
		packet[15] = (byte) ((pos5 >>> 8) & 0xff);
		packet[16] = (byte) (pos6 & 0xff);					//channel 6
		packet[17] = (byte) ((pos6 >>> 8) & 0xff);
		packet[18] = (byte) (time & 0xff);					//time
		packet[19] = (byte) ((time >>> 8) & 0xff);
		packet[20] = crc(packet);							//Checksum
		
		packet[21] = ETX0;	packet[22] = ETX1;
		return packet;
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
	public static byte[] servoNonTimeCtrlAll(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
	{
		  byte[] packet = new byte[21];
		  packet[0] = STX0;	  packet[1] = STX1;
		  packet[2] = 1;	  packet[3] = 0;
		  
		  packet[4]  = ALL_SERVO_CTRL;                //DID
		  packet[5]  = 12;                            //LEN
		  packet[6]  = (byte)(pos1 & 0xff);           //motor 1
		  packet[7]  = (byte)((pos1 >>> 8) & 0xff);             
		  packet[8]  = (byte)(pos2 & 0xff);           //motor 2
		  packet[9]  = (byte)((pos2 >>> 8) & 0xff);
		  packet[10] = (byte)(pos3 & 0xff);           //motor 3
		  packet[11] = (byte)((pos3 >>> 8) & 0xff);
		  packet[12] = (byte)(pos4 & 0xff);           //motor 4
		  packet[13] = (byte)((pos4 >>> 8) & 0xff);
		  packet[14] = (byte)(pos5 & 0xff);           //motor 5
		  packet[15] = (byte)((pos5 >>> 8) & 0xff);           
		  packet[16] = (byte)(pos6 & 0xff);           //motor 6
		  packet[17] = (byte)((pos6 >>> 8) & 0xff);            
		  packet[18] = crc(packet);                   //Checksum
		  
		  packet[19] = ETX0;  packet[20] = ETX1;
		  return packet;
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
	public static byte[] lcdDisplayPMS(String bmpFileName)
	{
		return null;
		// TODO Auto-generated method stub
	}
}
