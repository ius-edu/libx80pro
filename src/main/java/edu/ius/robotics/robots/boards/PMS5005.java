package edu.ius.robotics.robots.boards;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class PMS5005
{
	/*
	 * Java implmentation written by Jesse Riddle and Colton Jenkins, with
	 * significant documentation taken from the DrRobot WiRobot SDK Application
	 * Programming Interface (API) Reference Manual - (For MS Windows) Version:
	 * 1.0.8 Feb. 2004 by DrRobot, Inc. and some parts from the DrRobot Java
	 * motion control demo program.
	 */
	
	/*
	 * The structure for a packet to the PMS5005 is the following: STX0 = 94
	 * (always) STX1 = 2 (always) RID = 1 (generally) Reserved = 0 (generally)
	 * DID (Data ID) = <DID> (your choice of DID) LENGTH = <len> (Length from
	 * next element to CRC byte) DATA = <data> (may be more than 1 byte in
	 * length) CHECKSUM = <cksum> (use crc() method to calculate on cmd) ETX0 =
	 * 94 (always) ETX1 = 13 (always)
	 */
	
	/* Packet Info */
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int DID_OFFSET = 4;
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 94; // 0x5E
	public static final byte STX1 = 2; // 0x02
	public static final byte ETX0 = 94; // 0x5E
	public static final byte ETX1 = 13; // 0x13
	
	/* RID */
	public static final byte RID_HOST = 0x00;
	public static final byte RID_PMS5005 = 0x01;

	/* RESERVED */
	public static final byte RESERVED = 0x00;
	
	/* FLAGS */
	public static final byte SINGLE_CHANNEL_TIMED_FLAG = 0x06;
	
	/* LCD Display */
	public static final byte FRAME_LENGTH = 64;
	
	/* Data ID (DID) descriptor listing */
	public static final byte POSITION_CTRL = 3;
	public static final byte ALL_POSITION_CTRL = 4;
	public static final byte PWM_CTRL = 5;
	public static final byte ALL_PWM_CTRL = 6;
	public static final byte PARAM_SET = 7;
	
	/* SubCommands under PARAM_SET */
	public static final byte DC_POSITION_PID = 7; // Position PID Control
	public static final byte DC_VELOCITY_PID = 8; // Velocity PID Control
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
	// to use as ubyte: (byte)(SETUP_COM & 0xFF)
	public static final short SETUP_COM = 255;
	/* End Data ID (DID) descriptor listing */
	
	public static final byte PWM_CTRL_MODE = 0x00;
	public static final byte POSITION_CTRL_MODE = 0x01;
	public static final byte VELOCITY_CTRL_MODE = 0x02;
	
	public static final int SINGLE_POT_SENSOR_USAGE = 0x00;
	public static final int DUAL_POT_SENSOR_USAGE = 0x01;
	public static final int ENCODER_SENSOR_USAGE = 0x02;
	
	public static final byte KP_ID = 1; // progressive id
	public static final byte KD_ID = 2; // derivative id
	public static final byte KI_ID = 3; // integral id
	
	public static final short NON_CTRL_CMD = (short) 0xffff; // No Control command
	public static final short NO_CTRL = (short) 0x8000; // 32768
	
	public static final int MAX_PWM_L = 32767;
	public static final int MAX_PWM_R = 0;
	
	public static final int PWM_N = 16383;
	//public static final int PWM_O = 8000;
	public static final int PWM_O = 12000;
	public static final int DUTY_CYCLE_UNIT = 8383;
	
	/* Sensor Data Offsets */
	public static final int ULTRASONIC_OFFSET_WITH_HEADER = 0 + HEADER_LENGTH;
	public static final int ENCODER_PULSE_OFFSET_WITH_HEADER = 24 + HEADER_LENGTH;
	public static final int ENCODER_SPEED_OFFSET_WITH_HEADER = 32 + HEADER_LENGTH;
	public static final int STANDARD_IR_RANGE_OFFSET_WITH_HEADER = 24 + HEADER_LENGTH;
	public static final int CUSTOM_IR_RANGE_OFFSET_WITH_HEADER = 4 + HEADER_LENGTH; // CustomAD3
	public static final int HUMAN_ALARM_OFFSET_WITH_HEADER = 6 + HEADER_LENGTH;
	public static final int HUMAN_MOTION_OFFSET_WITH_HEADER = 8 + HEADER_LENGTH;
	public static final int TILTING_X_OFFSET_WITH_HEADER = 14 + HEADER_LENGTH;
	public static final int TILTING_Y_OFFSET_WITH_HEADER = 16 + HEADER_LENGTH;
	public static final int ENCODER_DIRECTION_OFFSET_WITH_HEADER = 32 + HEADER_LENGTH;
	public static final int MOTOR_SPEED_OFFSET_WITH_HEADER = 26 + HEADER_LENGTH;
	public static final int CUSTOM_AD_OFFSET_WITH_HEADER = 0 + HEADER_LENGTH;
	public static final int TEMPERATURE_AD_OFFSET_WITH_HEADER = 22 + HEADER_LENGTH;
	public static final int OVERHEAT_SENSOR_OFFSET_WITH_HEADER = 18 + HEADER_LENGTH;
	public static final int INFRARED_COMMAND_OFFSET_WITH_HEADER = 26 + HEADER_LENGTH;
	public static final int BATTERY_SENSOR_OFFSET_WITH_HEADER = 30 + HEADER_LENGTH;
	public static final int REFERENCE_VOLTAGE_OFFSET_WITH_HEADER = 36 + HEADER_LENGTH;
	public static final int POTENTIOMETER_POWER_OFFSET_WITH_HEADER = 38 + HEADER_LENGTH;
	public static final int POTENTIOMETER_SENSOR_OFFSET_WITH_HEADER = 0 + HEADER_LENGTH;
	public static final int MOTOR_CURRENT_SENSOR_OFFSET_WITH_HEADER = 12 + HEADER_LENGTH;
	
	/* Sensor Data Size */
	public static final int MOTOR_SENSOR_DATA_LENGTH = 1024; // 34
	public static final int CUSTOM_SENSOR_DATA_LENGTH = 1024; // 37
	public static final int STANDARD_SENSOR_DATA_LENGTH = 1024; // 33
	
	/* Magic numbers */
	public static final int MAX_PULSE_WIDTH = 32767;
	public static final int MIN_PULSE_WIDTH = -32768;
	public static final int NEUTRAL_PULSE_WIDTH = 0;
	
	/**
	 * Calculates a valid CRC value to be used in order to check the integrity
	 * of the contents of a request packet.
	 * 
	 * @param buf
	 *            is the buffer from which the CRC value will be calculated.
	 * 
	 * @return The CRC value calculated from the given buffer.
	 */
	public static byte calcCRC(byte[] buf)
	{
		byte shift_reg, sr_lsb, data_bit, v;
		byte fb_bit;
		short z;
		shift_reg = 0; // initialize the shift register
		z = (short) (buf.length - 3);// Don't include CRC and ETX (z=length-3)
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
				shift_reg = (byte) ((shift_reg & 0xff) >> 1);
				
				if (fb_bit == 1) 
				{
					shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xff);
				}
				
				v = (byte) ((v & 0xff) >> 1);
			}
		}
		
		return shift_reg;
	}
	
	public static byte[] ack()
	{
		byte[] msg = new byte[9];
		
		msg[0] = STX0;
		msg[1] = STX1;
		msg[2] = RID_PMS5005;
		msg[3] = RESERVED;
		msg[4] = 0x01; // pong
		msg[5] = calcCRC(msg);
		msg[6] = ETX0;
		msg[7] = ETX1;
		
		return msg;
	}
	
	/**
	 * Sends a request command to the Sensing and Motion Controller (PMS5005) in
	 * order to get the sensor data related to motor control.
	 * 
	 * @param packetNumber
	 *            describes how many times sampled data should be sent
	 *            packetNumber == 0 -- Stop sending the sensor data packets.
	 *            packetNumber == 1 -- Send sensor data packets continuously
	 *            until being asked to stop (default). packetNumber > 0 -- Send
	 *            n = packetNumber packets of sensor data and then stop sending.
	 * 
	 *            Please Note: Though adjustable, the maximum refresh rate for
	 *            the PMS5005 is 20Hz or 50ms (default).
	 * 
	 *            See Also: setMotorSensorPeriod
	 */
	public static byte[] motorSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = calcCRC(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Sends a request command to the Sensing and Motion Controller (PMS5005) in
	 * order to get all the standard sensor data.
	 * 
	 * @param packetNumber
	 *            describes how many times sampled data should be sent.
	 *            packetNumber == 0 -- Stop sending the sensor data packets.
	 *            packetNumber == 1 -- Send sensor data packets continuously
	 *            until being asked to stop. packetNumber > 0 -- Send n =
	 *            packetNumber packets of sensor data and then stop sending.
	 * 
	 *            Please Note: Though adjustable, the maximum refresh rate for
	 *            the PMS5005 is 20Hz or 50ms (default).
	 * 
	 *            See Also: setStandardSensorPeriod
	 */
	public static byte[] standardSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = calcCRC(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Sends a request command to the Sensing and Motion Controller (PMS5005) in
	 * order to get all custom sensor data.
	 * 
	 * @param packetNumber
	 *            describes how many times sampled data should be sent.
	 *            packetNumber == 0 -- Stop sending the sensor data packets.
	 *            packetNumber == 1 -- Send sensor data packets continuously
	 *            until being asked to stop (default). packetNumber > 0 -- Send
	 *            n = packetNumber packets of sensor data and then stop sending.
	 * 
	 *            Please Note: Though adjustable, the maximum refresh rate for
	 *            the PMS5005 is 20Hz or 50ms (default).
	 * 
	 *            See Also: setCustomSensorPeriod
	 */
	public static byte[] customSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = calcCRC(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Sends a request command to the Sensing and Motion Controller (PMS5005) in
	 * order to get all sensor data.
	 * 
	 * @param packetNumber
	 *            describes how many times sampled data should be sent.
	 *            packetNumber == 0 -- Stop sending the sensor data packets.
	 *            packetNumber == 1 -- Send sensor data packets continuously
	 *            until being asked to stop. packetNumber > 0 -- Send n =
	 *            packetNumber packets of sensor data and then stop sending.
	 * 
	 *            Please note: Though adjustable, the maximum refresh rate for
	 *            the PMS5005 is 20Hz or 50ms (default).
	 * 
	 *            See Also: setAllSensorPeriod
	 */
	public static byte[] allSensorRequest(short packetNumber)
	{
		byte[] cmd = new byte[10];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = (byte) (packetNumber & 0xff);
		cmd[7] = calcCRC(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Enables batch updating of motor-related sensor packets.
	 * 
	 * Please note: 1) The latest request setting of the packet number and the
	 * update rates are used. 2) By default, "all sensor data sending" is
	 * enabled.
	 * 
	 * @see motorSensorRequest
	 */
	public static byte[] enableMotorSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = calcCRC(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Enables batch updating of standard sensor packets.
	 * 
	 * Please note: 1) The latest request setting of the packet number and the
	 * update rates are used. 2) By default, "all sensor data sending" is
	 * enabled.
	 * 
	 * @see disableMotorSensorSending
	 */
	public static byte[] enableStandardSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_STANDARD_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = calcCRC(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Enables batch updating of custom sensor packets.
	 * 
	 * Please note: 1) The latest request setting of the packet number and the
	 * update rates are used. 2) By default, "all sensor data sending" is
	 * enabled.
	 * 
	 * @see disableStandardSensorSending
	 */
	public static byte[] enableCustomSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = calcCRC(cmd);
		cmd[7] = ETX0;
		cmd[8] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Enables batch updating of all sensor packets.
	 * 
	 * Please note: 1) The latest request setting of the packet number and the
	 * update rates are used. 2) By default, "all sensor data sending" is
	 * enabled.
	 * 
	 * @see disableCustomSensorSending
	 */
	public static byte[] enableAllSensorSending()
	{
		byte[] cmd = new byte[9];
		
		cmd[0] = STX0;
		cmd[1] = STX1;
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 0; // len
		cmd[6] = calcCRC(cmd);
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
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_MOTOR_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; // (byte) (0 & 0xff);
		cmd[7] = calcCRC(cmd);
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
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_STANDARD_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; // (byte) (0 & 0xff);
		cmd[7] = calcCRC(cmd);
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
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_CUSTOM_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; // (byte) (0 & 0xff);
		cmd[7] = calcCRC(cmd);
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
		cmd[2] = RID_PMS5005;
		cmd[3] = RESERVED;
		cmd[4] = GET_ALL_SENSOR_DATA;
		cmd[5] = 1; // len
		cmd[6] = 0; // (byte) (0 & 0xff);
		cmd[7] = calcCRC(cmd);
		cmd[8] = ETX0;
		cmd[9] = ETX1;
		
		return cmd;
	}
	
	/**
	 * Sets the refresh rate for batch updating by motor-related sensor packets.
	 * 
	 * @param timePeriod
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
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
	 * @param timePeriod
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
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
	 * @param timePeriod
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
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
	 * @param timePeriod
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
	 * 
	 * @see allSensorRequest
	 */
	public static byte[] setAllSensorPeriod(short timePeriod)
	{
		// TODO stub
		return null;
	}

	/**
	 * Sets the 8-channel custom digital outputs.
	 * 
	 * @param ival
	 *            -- only the lower 8 bits are valid and can change the
	 *            corresponding outputs of the 8 channels. The MSB of the lower
	 *            byte represents channel #8 and LSB of the lower byte
	 *            represents channel #1.
	 */
	public static byte[] setCustomDOut(byte ival)
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
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * 
	 * @param polarity
	 *            1 or -1
	 */
	public static byte[] setMotorPolarity(byte channel, byte polarity)
	{
		byte[] packet = new byte[12];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PARAM_SET; // DID
		packet[5] = 3; // LEN
		packet[6] = DC_SENSOR_USAGE; // Subcommand
		packet[7] = channel; // 0-L | 1=R
		packet[8] = polarity; // polarity 1 | -1
		packet[9] = calcCRC(packet); // Checksum
		packet[10] = ETX0;
		packet[11] = ETX1;
		
		return packet;
	}
	
	/**
	 * Enables the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * 
	 * @deprecated
	 * 
	 * @see resumeDCMotor
	 */
	public static byte[] enableDCMotor(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 1; // 1 = Enable/Resume
		packet[7] = channel; // 0=L | 1=R
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Disables the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * 
	 * @deprecated
	 * 
	 * @see suspendDCMotor
	 */
	public static byte[] disableDCMotor(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 0; // 0 = Disable/Suspend
		packet[7] = channel; // 0=L | 1=R
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Resumes the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 */
	public static byte[] resumeDCMotor(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 1; // resume
		packet[7] = channel; // 0=L | 1=R
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Suspends the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * 
	 *            All motor control channels are initially suspended at boot-up.
	 */
	public static byte[] suspendDCMotor(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 0; // SUSPEND
		packet[7] = channel; // 0=L | 1=R
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sets up the PID control parameters of the specified DC motor channel for
	 * position control.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * @param Kp
	 *            proportional gain (default is 50)
	 * @param Kd
	 *            derivative gain (default is 5)
	 * @param Ki_x100
	 *            the desired integral gain * 100. when Ki_100 = 100, the actual
	 *            integral control term is Ki = 1. Ki_x100 has a range of 0 to
	 *            25599, where 0 means no integral control (default).
	 * 
	 * @see setDCMotorControlMode
	 */
	public static byte[] setDCMotorPositionControlPID(byte channel, short kp, short kd, short ki)
	{
		byte[] packet = new byte[20];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PARAM_SET; // DID
		packet[5] = 11; // LEN
		packet[6] = DC_POSITION_PID; // Subcommand
		packet[7] = channel; // 0=L | 1=R
		packet[8] = KP_ID; // Proportional gain
		packet[9] = (byte) (kp & 0xff);
		packet[10] = (byte) ((kp >> 8) & 0xff);
		packet[11] = KD_ID; // Derivative gain
		packet[12] = (byte) (kd & 0xff);
		packet[13] = (byte) ((kd >> 8) & 0xff);
		packet[14] = KI_ID; // Integral gain
		packet[15] = (byte) (ki & 0xff);
		packet[16] = (byte) ((ki >> 8) & 0xff);
		packet[17] = calcCRC(packet); // Checksum
		packet[18] = ETX0;
		packet[19] = ETX1;
		
		return packet;
	}
	
	public static byte[] setDCMotorVelocityControlPID(byte channel, int kp, int kd, int ki)
	{
		byte[] packet = new byte[20];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PARAM_SET; // DID
		packet[5] = 11; // LEN
		packet[6] = DC_VELOCITY_PID; // Subcommand
		packet[7] = channel; // 0=L | 1=R
		packet[8] = KP_ID; // Proportional gain
		packet[9] = (byte) (kp & 0xff);
		packet[10] = (byte) ((kp >> 8) & 0xff);
		packet[11] = KD_ID; // Derivative gain
		packet[12] = (byte) (kd & 0xff);
		packet[13] = (byte) ((kd >> 8) & 0xff);
		packet[14] = KI_ID; // Integral gain
		packet[15] = (byte) (ki & 0xff);
		packet[16] = (byte) ((ki >> 8) & 0xff);
		packet[17] = calcCRC(packet); // Checksum
		packet[18] = ETX0;
		packet[19] = ETX1;
		
		return packet;
	}
	
	/**
	 * This filtering feature is still under development. All data will be
	 * treated as raw data.
	 */
	public static byte[] setDCMotorSensorFilter(byte channel, short filterMethod)
	{
		return null;
	}
	
	/**
	 * Set the sensor type for the specified DC motor control channel on the
	 * Sensing and Motion Controller (PMS5005).
	 * 
	 * The available sensor types are single potentiometer, dual potentiometers,
	 * and quadrature encoder. The single potentiometer sensor is for the
	 * control of robot joint with limited rotation range (0 degrees to 332
	 * degrees). The dual potentiometers and the quadrature sensor are for use
	 * with continuously rotating joints (e.g. wheels).
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5 for single potentiometer sensor channel 0,
	 *            1, or 2 for dual potentiometer sensor channel 0 or 1 for
	 *            quadrature encoder
	 * @param sensorType
	 *            0 -- single potentiometer sensor 1 -- dual potentiometer
	 *            sensor 2 -- quadrature encoder
	 * 
	 *            Please note 1) The electrical angular range of the
	 *            potentiometer position sensor is 0 degrees to 332 degrees and
	 *            the corresponding mechanical rotation range is 14 degrees to
	 *            346 degrees, with the 180 degree position defined as the
	 *            sensor's physical middle position. 2) Each DC motor channel
	 *            for dual potentiometer sensors utilizes two potentiometer
	 *            channels. DC motor channel 0
	 * 
	 * @see getSensorPot
	 */
	public static byte[] setDCMotorSensorUsage(byte channel, byte sensorType)
	{
		byte[] packet = new byte[12];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PARAM_SET; // DID
		packet[5] = 3; // LEN
		packet[6] = DC_SENSOR_USAGE; // Subcommand
		packet[7] = channel; // 0-5 = Single Potentiometer, 0-2 = Dual Potentiometer, 0-1 = Encoder
		packet[8] = sensorType; // 0x00 = Single Potentiometer, 0x01 = Dual Potentiometer, 0x02 = Encoder
		packet[9] = calcCRC(packet); // Checksum
		packet[10] = ETX0;
		packet[11] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sets the control mode of the specified DC motor control channel on the
	 * Sensing and Motion Controller (PMS5005). The available control modes are
	 * open-loop PWM control, closed-loop position control, and closed- loop
	 * velocity control.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param controlMode
	 *            0 == open-loop PWM control, 1 == closed-loop position control,
	 *            2 == closed-loop velocity control
	 * 
	 * @see setDCMotorPositionControlPID
	 * @see setDCMotorVelocityControlPID
	 */
	public static byte[] setDCMotorControlMode(byte channel, byte controlMode)
	{
		byte[] packet = new byte[12];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PARAM_SET; // DID
		packet[5] = 3; // LEN
		packet[6] = DC_CTRL_MODE; // Subcommand
		packet[7] = channel; // channel 0-5
		packet[8] = controlMode; // 0 = open, 1 = closed position, 2 = closed // velocity
		packet[9] = calcCRC(packet); // Checksum
		packet[10] = ETX0;
		packet[11] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the position control command to the specified motion control
	 * channel on the Sensing and Motion Controller (PMS5005). The command
	 * includes the target position and the target time period to execute the
	 * command. The current trajectory planning method with time control is
	 * linear.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pos
	 *            Target position value
	 * @param timePeriod
	 *            Executing time in milliseconds
	 */
	public static byte[] setDCMotorPosition(byte channel, short pos, short timePeriod)
	{
		byte[] packet = new byte[14];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = POSITION_CTRL; // DID
		packet[5] = 5; // LEN
		packet[6] = channel; // Channel 0-5
		packet[7] = (byte) (pos & 0xff); // cmdValue
		packet[8] = (byte) ((pos >> 8) & 0xff);
		packet[9] = (byte) (timePeriod & 0xff); // time
		packet[10] = (byte) ((timePeriod >> 8) & 0xff);
		packet[11] = calcCRC(packet); // Checksum
		packet[12] = ETX0;
		packet[13] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the position control command to the specified motion control
	 * channel on the Sensing and Motion Controller (PMS5005). The command
	 * includes the target position but no time period specified to execute the
	 * command. The motion controller will drive the motor to the target
	 * position at the maximum speed.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pos
	 *            Target position value
	 * 
	 *            1) Motor will be enabled automatically by the system when this
	 *            command is received. 2) No velocity is available for motor
	 *            channel using single potentiometer sensor. 3) The unit of the
	 *            velocity is (Position change in A/D sampling data) / second
	 *            when using dual potentiometer sensor for rotational postion
	 *            measurement and pulse/second when using quadrature encoder.
	 * 
	 * @see DCMotorVelocityTimeCtrl
	 * @see getSensorPot
	 */
	public static byte[] setDCMotorPosition(byte channel, short pos)
	{
		byte[] packet = new byte[12];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = POSITION_CTRL; // DID
		packet[5] = 3; // LEN
		packet[6] = channel; // channel 0-5
		packet[7] = (byte) (pos & 0xff); // cmdValue
		packet[8] = (byte) ((pos >> 8) & 0xff);
		packet[9] = calcCRC(packet); // Checksum
		packet[10] = ETX0;
		packet[11] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to the specified motion control channel on
	 * the Sensing and Motion Controller (PMS5005). The command includes the
	 * target pulse width value and the time period to execute the command. The
	 * current trajectory planning method for time control is linear.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pulseWidth
	 *            Target pulse width value
	 * @param timePeriod
	 *            Executing time in milliseconds
	 * 
	 *            1) The specified channel (motor) will be enabled automatically
	 *            by the system when this command is received. 2) Target pulse
	 *            width value range is 0 to 32767 (0x7FFF), corresponding to the
	 *            duty cycle of 0 to 100% linearly. 3) A pulse width value of
	 *            16383 means 50% duty cycle, putting the motor in the stop
	 *            (neutral) stage. Any value in between 16384 -> 32767 will cause
	 *            the motor to turn clockwise (facing the front side of the
	 *            motor) and any value in between 16362 -> 0 will cause the motor
	 *            to turn counter-clockwise. 16383 is neutral.
	 * 
	 */
	public static byte[] setDCMotorPulse(byte channel, short pulseWidth, short timePeriod)
	{
		byte[] packet = new byte[15];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PWM_CTRL; // DID
		packet[5] = 6; // LEN
		packet[6] = channel; // Channel 0-5
		packet[7] = (byte) (pulseWidth & 0xff); // cmdValue
		packet[8] = (byte) ((pulseWidth >> 8) & 0xff);
		packet[9] = (byte) SINGLE_CHANNEL_TIMED_FLAG; // we have a flag
		packet[10] = (byte) (timePeriod & 0xff); // time
		packet[11] = (byte) ((timePeriod >> 8) & 0xff);
		packet[12] = calcCRC(packet); // Checksum
		packet[13] = ETX0;
		packet[14] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to the specified motion control channel on
	 * the Sensing and Motion Controller (PMS5005). The command includes the
	 * target pulse width value without an execution time period specified. The
	 * motion controller will set the PWM output of this channel to the target
	 * value immediately.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pulseWidth
	 *            Target pulse width value
	 * 
	 *            1) The specified channel (motor) will be enabled automatically
	 *            by the system when this command is received. 2) Target pulse
	 *            width value range is 0 to 32767 (0x7FFF), corresponding to the
	 *            duty cycle of 0 to 100% linearly. 3) A pulse width value of
	 *            16383 means 50% duty cycle, putting the motor in the "Stop"
	 *            stage. Any value between 16364 -> 32767 will cause the motor to
	 *            turn clockwise (facing the front side of the motor) and any
	 *            value in between 16362 -> 0  will cause the motor to turn
	 *            counter-clockwise. 16383 is neutral.
	 * 
	 * @see DCMotorPwmTimeCtrl
	 */
	public static byte[] setDCMotorPulse(byte channel, short pulseWidth)
	{
		byte[] packet = new byte[12];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = PWM_CTRL; // DID
		packet[5] = 3; // LEN
		packet[6] = channel; // Channel 0-5
		packet[7] = (byte) (pulseWidth & 0xff); // cmdValue
		packet[8] = (byte) ((pulseWidth >> 8) & 0xff);
		packet[9] = calcCRC(packet); // Checksum
		packet[10] = ETX0;
		packet[11] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the position control command to all 6 DC motor control channels on
	 * the sensing and motion controller (PMS5005) at the same time. The command
	 * includes the target positions and the time period to execute the command.
	 * The current trajectory planning method for time control is linear.
	 * 
	 * @param pos1
	 *            Target position for channel #1
	 * @param pos2
	 *            Target position for channel #2
	 * @param pos3
	 *            Target position for channel #3
	 * @param pos4
	 *            Target position for channel #4
	 * @param pos5
	 *            Target position for channel #5
	 * @param pos6
	 *            Target position for channel #6
	 * @param timePeriod
	 *            Execution time in milliseconds
	 * 
	 *            1) All DC Motors will be enabled automatically by the system
	 *            when this command is received. 2) Target position value is the
	 *            A/D sampling data range 0 to 4095 when using single
	 *            potentiometer, 0-4428 when using dual potentiometers. 3)
	 *            Please refer to the description of getSensorPot for data
	 *            conversion between angular values and the A/D sampling data
	 *            values. 4) When using the encoder as sensor input, the target
	 *            position value is the pulse count in the range of 0-32767. 5)
	 *            When omitting motor channels from control, the command value
	 *            should be set to -32768 (0x8000), which implies NO_CTRL.
	 * 
	 * @see getSensorPot
	 * @see DCMotorPositionTimeCtrl
	 */
	public static byte[] setAllDCMotorPositions(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6, short timePeriod)
	{
		byte[] packet = new byte[23];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_POSITION_CTRL; // DID
		packet[5] = 14; // LEN
		packet[6] = (byte) (pos1 & 0xff); // channel 1
		packet[7] = (byte) ((pos1 >> 8) & 0xff);
		packet[8] = (byte) (pos2 & 0xff); // channel 2
		packet[9] = (byte) ((pos2 >> 8) & 0xff);
		packet[10] = (byte) (pos3 & 0xff); // channel 3
		packet[11] = (byte) ((pos3 >> 8) & 0xff);
		packet[12] = (byte) (pos4 & 0xff); // channel 4
		packet[13] = (byte) ((pos4 >> 8) & 0xff);
		packet[14] = (byte) (pos5 & 0xff); // channel 5
		packet[15] = (byte) ((pos5 >> 8) & 0xff);
		packet[16] = (byte) (pos6 & 0xff); // channel 6
		packet[17] = (byte) ((pos6 >> 8) & 0xff);
		packet[18] = (byte) (timePeriod & 0xff); // time
		packet[19] = (byte) ((timePeriod >> 8) & 0xff);
		packet[20] = calcCRC(packet); // Checksum
		packet[21] = ETX0;
		packet[22] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the position control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target positions without a specified time period for
	 * execution. The motion controller will drive the motor to reach the target
	 * position with maximum effort.
	 * 
	 * @param pos1
	 *            Target position for channel #1
	 * @param pos2
	 *            Target position for channel #2
	 * @param pos3
	 *            Target position for channel #3
	 * @param pos4
	 *            Target position for channel #4
	 * @param pos5
	 *            Target position for channel #5
	 * @param pos6
	 *            Target position for channel #6
	 * 
	 *            1) All DC Motors will be enabled automatically by the system
	 *            when this command is received. 2) Target position value is the
	 *            A/D sampling data range 0 to 4095 when using single
	 *            potentiometer, 0-4428 when using dual potentiometers. 3)
	 *            Please refer to the description of getSensorPot for data
	 *            conversion between angular values and the A/D sampling data
	 *            values. 4) When using the encoder as sensor input, the target
	 *            position value is the pulse count in the range of 0-32767. 5)
	 *            When omitting motor channels from control, the command value
	 *            should be set to -32768 (0x8000), which implies NO_CTRL.
	 * 
	 * @see getSensorPot
	 * @see DCMotorPositionNonTimeCtrl
	 */
	public static byte[] setAllDCMotorPositions(short pos1, short pos2, short pos3, short pos4, short pos5, short pos6)
	{
		byte[] packet = new byte[21];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_POSITION_CTRL; // DID
		packet[5] = 12; // LEN
		packet[6] = (byte) (pos1 & 0xff); // channel 1
		packet[7] = (byte) ((pos1 >> 8) & 0xff);
		packet[8] = (byte) (pos2 & 0xff); // channel 2
		packet[9] = (byte) ((pos2 >> 8) & 0xff);
		packet[10] = (byte) (pos3 & 0xff); // channel 3
		packet[11] = (byte) ((pos3 >> 8) & 0xff);
		packet[12] = (byte) (pos4 & 0xff); // channel 4
		packet[13] = (byte) ((pos4 >> 8) & 0xff);
		packet[14] = (byte) (pos5 & 0xff); // channel 5
		packet[15] = (byte) ((pos5 >> 8) & 0xff);
		packet[16] = (byte) (pos6 & 0xff); // channel 6
		packet[17] = (byte) ((pos6 >> 8) & 0xff);
		packet[18] = calcCRC(packet); // Checksum
		packet[19] = ETX0;
		packet[20] = ETX1;
		return packet;
	}
	
	/**
	 * Sends the velocity control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target velocities and the time period to execute the
	 * command. The trajectory planning method for time control is linear.
	 * 
	 * @param v0
	 *            Target velocity for channel #1
	 * @param v1
	 *            Target velocity for channel #2
	 * @param v2
	 *            Target velocity for channel #3
	 * @param v3
	 *            Target velocity for channel #4
	 * @param v4
	 *            Target velocity for channel #5
	 * @param v5
	 *            Target velocity for channel #6
	 * @param timePeriod
	 *            Execution time in milliseconds
	 * 
	 *            1) Motor will be enabled automatically by the system when this
	 *            command is received. 2) No velocity control is available for a
	 *            motor channel operating in single potentiometer mode. 3) The
	 *            unit of the velocity is (Position change in A/D sampling data)
	 *            / second when using dual potentiometer sensors for rotational
	 *            position measurements and pulse/second when using quadrature
	 *            encoder. 4) Please refer to the description of getSensorPot
	 *            for data conversion between angular values and the A/D
	 *            sampling data values. 5) When omitting motors from control,
	 *            send the command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 * @see DCMotorVelocityTimeCtrl
	 */
	public static byte[] setAllDCMotorVelocities(short v0, short v1, short v2, short v3, short v4, short v5, short timePeriod)
	{
		byte[] packet = new byte[23];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_VELOCITY_CTRL; // DID
		packet[5] = 14; // LEN
		packet[6] = (byte) (v0 & 0xff); // MOTOR 1
		packet[7] = (byte) ((v0 >> 8) & 0xff);
		packet[8] = (byte) (v1 & 0xff); // MOTOR 2
		packet[9] = (byte) ((v1 >> 8) & 0xff);
		packet[10] = (byte) (v2 & 0xff); // MOTOR 3
		packet[11] = (byte) ((v2 >> 8) & 0xff);
		packet[12] = (byte) (v3 & 0xff); // MOTOR 4
		packet[13] = (byte) ((v3 >> 8) & 0xff);
		packet[14] = (byte) (v4 & 0xff); // MOTOR 5
		packet[15] = (byte) ((v4 >> 8) & 0xff);
		packet[16] = (byte) (v5 & 0xff); // MOTOR 6
		packet[17] = (byte) ((v5 >> 8) & 0xff);
		packet[18] = (byte) (timePeriod & 0xff); // time
		packet[19] = (byte) ((timePeriod >> 8) & 0xff);
		packet[20] = calcCRC(packet); // Checksum
		packet[21] = ETX0;
		packet[22] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the velocity control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target velocities without specifying an execution time
	 * period. The motion controller will drive the motor to achieve the target
	 * velocity with maximum effort.
	 * 
	 * @param v0
	 *            Target velocity for channel #1
	 * @param v1
	 *            Target velocity for channel #2
	 * @param v2
	 *            Target velocity for channel #3
	 * @param v3
	 *            Target velocity for channel #4
	 * @param v4
	 *            Target velocity for channel #5
	 * @param v5
	 *            Target velocity for channel #6
	 * 
	 *            1) Motor will be enabled automatically by the system when this
	 *            command is received. 2) No velocity control is available for a
	 *            motor channel operating in single potentiometer mode. 3) The
	 *            unit of the velocity is (Position change in A/D sampling data)
	 *            / second when using dual potentiometer sensors for rotational
	 *            position measurements and pulse/second when using quadrature
	 *            encoder. 4) Please refer to the description of getSensorPot
	 *            for data conversion between angular values and the A/D
	 *            sampling data values. 5) When omitting motors from control,
	 *            send the command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 */
	public static byte[] setAllDCMotorVelocities(short v0, short v1, short v2, short v3, short v4, short v5)
	{
		byte[] packet = new byte[21];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_VELOCITY_CTRL; // DID
		packet[5] = 12; // LEN
		packet[6] = (byte) (v0 & 0xff); // MOTOR 1
		packet[7] = (byte) ((v0 >> 8) & 0xff);
		packet[8] = (byte) (v1 & 0xff); // MOTOR 2
		packet[9] = (byte) ((v1 >> 8) & 0xff);
		packet[10] = (byte) (v2 & 0xff); // MOTOR 3
		packet[11] = (byte) ((v2 >> 8) & 0xff);
		packet[12] = (byte) (v3 & 0xff); // MOTOR 4
		packet[13] = (byte) ((v3 >> 8) & 0xff);
		packet[14] = (byte) (v4 & 0xff); // MOTOR 5
		packet[15] = (byte) ((v4 >> 8) & 0xff);
		packet[16] = (byte) (v5 & 0xff); // MOTOR 6
		packet[17] = (byte) ((v5 >> 8) & 0xff);
		//packet[18] = (byte) 0x06; // we have a flag?
		packet[18] = calcCRC(packet); // Checksum
		packet[19] = ETX0;
		packet[20] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to all 6 DC motor control channels on the
	 * Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target PWM values and the time period for execution. The
	 * current trajectory planning method for time control is linear.
	 * 
	 * @param p0
	 *            Target PWM value (pulse width) for channel #1
	 * @param p1
	 *            Target PWM value (pulse width) for channel #2
	 * @param p2
	 *            Target PWM value (pulse width) for channel #3
	 * @param p3
	 *            Target PWM value (pulse width) for channel #4
	 * @param p4
	 *            Target PWM value (pulse width) for channel #5
	 * @param p5
	 *            Target PWM value (pulse width) for channel #6
	 * @param timePeriod
	 *            Execution time in milliseconds
	 * 
	 *            1) All channels (motors) will be enable automatically by the
	 *            system when this command is received. 2) Target pulse width
	 *            value range is 0 to 32767 (0x7FFF), corresponding to the duty
	 *            cycle of 0 to 100% linearly. 3) A pulse width value of 16383
	 *            means 50% duty cycle, putting the motor in the stop (neutral)
	 *            stage. Any value in between 16384 - 32767 will cause the motor
	 *            to turn clockwise (facing the front side of the motor) and any
	 *            value in between 0 - 16362 will cause the motor to turn
	 *            counter-clockwise. 4) When omitting motors from control, the
	 *            command value of -32768 (0x8000), should be sent. This implies
	 *            NO_CTRL.
	 */
	public static byte[] setAllDCMotorPulses(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
	{
		byte[] packet = new byte[23];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_PWM_CTRL; // DID
		packet[5] = 14; // LEN
		packet[6] = (byte) (p0 & 0xff); // MOTOR 1
		packet[7] = (byte) ((p0 >> 8) & 0xff);
		packet[8] = (byte) (p1 & 0xff); // MOTOR 2
		packet[9] = (byte) ((p1 >> 8) & 0xff);
		packet[10] = (byte) (p2 & 0xff); // MOTOR 3
		packet[11] = (byte) ((p2 >> 8) & 0xff);
		packet[12] = (byte) (p3 & 0xff); // MOTOR 4
		packet[13] = (byte) ((p3 >> 8) & 0xff);
		packet[14] = (byte) (p4 & 0xff); // MOTOR 5
		packet[15] = (byte) ((p4 >> 8) & 0xff);
		packet[16] = (byte) (p5 & 0xff); // MOTOR 6
		packet[17] = (byte) ((p5 >> 8) & 0xff);
		packet[18] = (byte) (timePeriod & 0xff); // time
		packet[19] = (byte) ((timePeriod >> 8) & 0xff);
		packet[20] = calcCRC(packet); // Checksum
		packet[21] = ETX0;
		packet[22] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to all 6 DC motor control channels on the
	 * Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target PWM values without a specified time period for
	 * execution. The motion controller will adjust the pulse width right away.
	 * 
	 * @param p0
	 *            Target PWM value (pulse width) for channel #1
	 * @param p1
	 *            Target PWM value (pulse width) for channel #2
	 * @param p2
	 *            Target PWM value (pulse width) for channel #3
	 * @param p3
	 *            Target PWM value (pulse width) for channel #4
	 * @param p4
	 *            Target PWM value (pulse width) for channel #5
	 * @param p5
	 *            Target PWM value (pulse width) for channel #6
	 * 
	 *            1) All channels (motors) will be enable automatically by the
	 *            system when this command is received. 2) Target pulse width
	 *            value range is 0 to 32767 (0x7FFF), corresponding to the duty
	 *            cycle of 0 to 100% linearly. 3) A pulse width value of 16383
	 *            means 50% duty cycle, putting the motor in the stop (neutral)
	 *            stage. Any value in between 16384 - 32767 will cause the motor
	 *            to turn clockwise (facing the front side of the motor) and any
	 *            value in between 0 - 16362 will cause the motor to turn
	 *            counter-clockwise. 4) When omitting motors from control, the
	 *            command value of -32768 (0x8000), should be sent. This implies
	 *            NO_CTRL.
	 */
	public static byte[] setAllDCMotorPulses(short p0, short p1, short p2, short p3, short p4, short p5)
	{
		byte[] packet = new byte[21];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_PWM_CTRL; // DID
		packet[5] = 12; // LEN
		packet[6] = (byte) (p0 & 0xff); // MOTOR 1
		packet[7] = (byte) ((p0 >> 8) & 0xff);
		packet[8] = (byte) (p1 & 0xff); // MOTOR 2
		packet[9] = (byte) ((p1 >> 8) & 0xff);
		packet[10] = (byte) (p2 & 0xff); // MOTOR 3
		packet[11] = (byte) ((p2 >> 8) & 0xff);
		packet[12] = (byte) (p3 & 0xff); // MOTOR 4
		packet[13] = (byte) ((p3 >> 8) & 0xff);
		packet[14] = (byte) (p4 & 0xff); // MOTOR 5
		packet[15] = (byte) ((p4 >> 8) & 0xff);
		packet[16] = (byte) (p5 & 0xff); // MOTOR 6
		packet[17] = (byte) ((p5 >> 8) & 0xff);
		packet[18] = calcCRC(packet); // Checksum
		packet[19] = ETX0;
		packet[20] = ETX1;
		
		return packet;
	}
	
	/**
	 * Enables the specified servo motor control channel.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * 
	 *            All servo motor channels are disable initially at system
	 *            startup. They need to be enabled explicitly before use.
	 * 
	 * @see disableServo
	 */
	public static byte[] enableServo(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 0; // 0 = Enable
		packet[7] = channel; // 6-11 SERVO
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Disables the specified servo motor control channel.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * 
	 *            All servo motor channels are disable initially at system
	 *            startup. They need to be enabled explicitly before use.
	 * 
	 * @see enableServo
	 */
	public static byte[] disableServo(byte channel)
	{
		byte[] packet = new byte[11];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = TOGGLE_DC_MOTORS; // DID
		packet[5] = 2; // LEN
		packet[6] = 0; // 0 = Disable
		packet[7] = channel; // 6-11 = SERVO
		packet[8] = calcCRC(packet); // Checksum
		packet[9] = ETX0;
		packet[10] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to the specified servo motor control
	 * channel on the Sensing and Motion Controller (PMS5005). The command
	 * includes the target position command and the time period to execute the
	 * command. The current trajectory planning method for time control is
	 * linear.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pulseWidth
	 *            Target Pulse Width (in milliseconds) * 2250
	 * @param timePeriod
	 *            Executing time in milliseconds
	 * 
	 *            Usually, a standard remote control servo motor expects to get
	 *            the specified pulse width every 20 seconds in order to hold
	 *            the corresponding angle position. The pulse width value in
	 *            milliseconds for 0 degrees, 90 degrees, and 180 degrees are
	 *            servo manufacturer and model dependent. They are around 1ms,
	 *            1.5ms, and 2.0ms respectively for most common servos.
	 *            Experiments are required to obtain the exact value for a
	 *            specific servo motor.
	 * 
	 */
	public static byte[] setServoPulse(byte channel, short pulseWidth, short timePeriod)
	{
		byte[] packet = new byte[15];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = SERVO_CTRL; // DID
		packet[5] = 6; // LEN
		packet[6] = channel; // channel
		packet[7] = (byte) (pulseWidth & 0xff); // command value low 8 bit
		packet[8] = (byte) ((pulseWidth >> 8) & 0xff); // high 8 bit
		packet[9] = SINGLE_CHANNEL_TIMED_FLAG; // flag
		packet[10] = (byte) (timePeriod & 0xff); // time low 8 bit
		packet[11] = (byte) ((timePeriod >> 8) & 0xff); // high 8 bit
		packet[12] = calcCRC(packet); // Checksum
		packet[13] = ETX0;
		packet[14] = ETX1;
		return packet;
	}
	
	/**
	 * Sends the PWM control command to the specified servo motor control
	 * channel on the Sensing and Motion Controller (PMS5005). The command
	 * includes the target position command without a specific time period for
	 * execution. The motion controller will send the desired pulse width to the
	 * servo motor right away.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * @param pulseWidth
	 *            Target pulse width (ms) * 2250
	 * 
	 * @see servoTimeCtrl
	 */
	public static byte[] setServoPulse(byte channel, short pulseWidth)
	{
		byte[] packet = new byte[13];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = SERVO_CTRL; // DID
		packet[5] = 4; // LEN
		packet[6] = channel; // channel
		packet[7] = (byte) (pulseWidth & 0xff); // command value low 8 bit
		packet[8] = (byte) ((pulseWidth >> 8) & 0xff); // high 8 bit
		packet[9] = SINGLE_CHANNEL_TIMED_FLAG; // flag
		packet[10] = calcCRC(packet); // Checksum
		packet[11] = ETX0;
		packet[12] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends PWM control commands to all 6 servo motor control channels
	 * on the Sensing and Motion Controller (PMS5005) at the same time.
	 * 
	 * The command includes the target position commands and the time period to
	 * execute the command. The current trajectory planning method for time
	 * control is linear.
	 * 
	 * @param p0
	 *            Target pulse width for channel #1 (Left Motor on X80Pro)
	 * @param p1
	 *            Target pulse width for channel #2 (-Right Motor on X80Pro)
	 * @param p2
	 *            Target pulse width for channel #3 (NO_CTRL on X80Pro)
	 * @param p3
	 *            Target pulse width for channel #4 (NO_CTRL on X80Pro)
	 * @param p4
	 *            Target pulse width for channel #5 (NO_CTRL on X80Pro)
	 * @param p5
	 *            Target pulse width for channel #6 (NO_CTRL on X80Pro)
	 * @param timePeriod
	 *            Executing time in milliseconds
	 * 
	 *            When omitting servo motors from control, please send the
	 *            command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 */
	public static byte[] setAllServoPulses(short p0, short p1, short p2, short p3, short p4, short p5, short timePeriod)
	{
		byte[] packet = new byte[23];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_SERVO_CTRL; // DID
		packet[5] = 14; // LEN
		packet[6] = (byte) (p0 & 0xff); // channel 1
		packet[7] = (byte) ((p0 >> 8) & 0xff);
		packet[8] = (byte) (p1 & 0xff); // channel 2
		packet[9] = (byte) ((p1 >> 8) & 0xff);
		packet[10] = (byte) (p2 & 0xff); // channel 3
		packet[11] = (byte) ((p2 >> 8) & 0xff);
		packet[12] = (byte) (p3 & 0xff); // channel 4
		packet[13] = (byte) ((p3 >> 8) & 0xff);
		packet[14] = (byte) (p4 & 0xff); // channel 5
		packet[15] = (byte) ((p5 >> 8) & 0xff);
		packet[16] = (byte) (p5 & 0xff); // channel 6
		packet[17] = (byte) ((p5 >> 8) & 0xff);
		packet[18] = (byte) (timePeriod & 0xff); // time
		packet[19] = (byte) ((timePeriod >> 8) & 0xff);
		packet[20] = calcCRC(packet); // Checksum
		packet[21] = ETX0;
		packet[22] = ETX1;
		
		return packet;
	}
	
	/**
	 * Sends the PWM control command to all 6 servo motor control channels
	 * on the Sensing and Motion Controller (PMS5005) at the same time.
	 * 
	 * The command includes the target position commands without specifying a
	 * time period in which to execute the command. The motion controller sends
	 * the desired pulse width to the servo motor right away.
	 * 
	 * @param p0
	 *            Target pulse width for channel #1 (Left Motor on X80Pro)
	 * @param p1
	 *            Target pulse width for channel #2 (-Right Motor on X80Pro)
	 * @param p2
	 *            Target pulse width for channel #3 (NO_CTRL on X80Pro)
	 * @param p3
	 *            Target pulse width for channel #4 (NO_CTRL on X80Pro)
	 * @param p4
	 *            Target pulse width for channel #5 (NO_CTRL on X80Pro)
	 * @param p5
	 *            Target pulse width for channel #6 (NO_CTRL on X80Pro)
	 * 
	 *            When omitting servo motors from control, please send the
	 *            command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 * @see servoNonTimeCtrl
	 */
	public static byte[] setAllServoPulses(short p0, short p1, short p2, short p3, short p4, short p5)
	{
		byte[] packet = new byte[21];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = ALL_SERVO_CTRL; // DID
		packet[5] = 12; // LEN
		packet[6] = (byte) (p0 & 0xff); // motor 1
		packet[7] = (byte) ((p0 >> 8) & 0xff);
		packet[8] = (byte) (p1 & 0xff); // motor 2
		packet[9] = (byte) ((p1 >> 8) & 0xff);
		packet[10] = (byte) (p2 & 0xff); // motor 3
		packet[11] = (byte) ((p2 >> 8) & 0xff);
		packet[12] = (byte) (p3 & 0xff); // motor 4
		packet[13] = (byte) ((p3 >> 8) & 0xff);
		packet[14] = (byte) (p4 & 0xff); // motor 5
		packet[15] = (byte) ((p4 >> 8) & 0xff);
		packet[16] = (byte) (p5 & 0xff); // motor 6
		packet[17] = (byte) ((p5 >> 8) & 0xff);
		packet[18] = calcCRC(packet); // Checksum
		packet[19] = ETX0;
		packet[20] = ETX1;
		
		return packet;
	}
	
	/**
	 * Displays the image data in the file bmpFileName (BMP format) on the
	 * graphic LCD connected to the Sensing and Motion Controller (PMS5005).
	 * 
	 * @param frameNumber
	 * 			serial frame number
	 * @param frame
	 * 			64 byte frame data to display
	 */
	public static byte[] setLCDDisplayPMS(byte frameNumber, byte[] frame) // LCD PMS
	{
		byte[] packet = new byte[73];
		
		packet[0] = STX0;
		packet[1] = STX1;
		packet[2] = RID_PMS5005;
		packet[3] = RESERVED;
		packet[4] = LCD_CTRL;
		packet[5] = 65; // LEN (1 frame slice byte + 64 bytes of bitmap image data)
		packet[6] = (byte) (frameNumber & 0xFF);
		for (int i = 0; i < FRAME_LENGTH; ++i) packet[7+i] = (byte) (frame[i] & 0xFF);
		packet[71] = calcCRC(packet);
		packet[72] = ETX0;
		packet[73] = ETX1;
		
		return packet;
	}
}
