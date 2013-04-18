package edu.ius.robotics.robots.boards;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class PMS5005
{
	/*
	 * Java implementation written by Jesse Riddle and Colton Jenkins, with
	 * significant documentation taken from the DrRobot WiRobot SDK Application
	 * Programming Interface (API) Reference Manual - (For MS Windows) Version:
	 * 1.0.8 Feb. 2004 by DrRobot, Inc. and some parts from the DrRobot Java
	 * motion control demo program.
	 */
	
	/*
	 * The structure for a packet to the PMS5005 is the following: STX0 = 94 (0x5E)
	 * (always) STX1 = 2 (0x02) (always) RID = 1 (generally) Reserved = 0 (generally)
	 * DID (Data ID) = <DID> (your choice of DID) LENGTH = <len> (Length from
	 * next element to CRC byte) DATA = <data> (may be more than 1 byte in
	 * length) CHECKSUM = <cksum> (use crc() method to calculate on cmd) ETX0 =
	 * 94 (0x5E) (always) ETX1 = 13 (0x13) (always)
	 */
	
	public static final ByteOrder BYTE_ORDER = ByteOrder.LITTLE_ENDIAN;
	public static final byte ENABLE = 1;
	public static final byte DISABLE = 0;
	public static final byte RESUME = 1;
	public static final byte SUSPEND = 0;
	public static final byte ENABLE_SERVO = 0;
	public static final byte DISABLE_SERVO = 1;
	
	/* Packet Info */
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int METADATA_SIZE = 9;
	public static final int RID_OFFSET = 2;
	public static final int DID_OFFSET = 4;
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 0x5E;
	public static final byte STX1 = 0x02;
	public static final byte ETX0 = 0x5E;
	public static final byte ETX1 = 0x13;
	
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
	public static final byte PING = 0x01;
	public static final byte ACK = 0x01;
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
	// to use as ubyte: (SETUP_COM & 0xFF)
	public static final int SETUP_COM = 255;
	/* End Data ID (DID) descriptor listing */
	
	public static final byte PROPORTIONAL_GAIN = 1; // progressive id
	public static final byte DERIVATIVE_GAIN = 2; // derivative id
	public static final byte INTEGRAL_GAIN = 3; // integral id
	
	/**
	 * Calculates a valid CRC value to be used in order to check the integrity
	 * of the contents of a request packet.
	 * 
	 * @param buffer
	 *            is the buffer from which the CRC value will be calculated.
	 * 
	 * @return The CRC value calculated from the given buffer.
	 */
	public static byte checksum(byte[] buffer)
	{
		byte shift_reg, sr_lsb, data_bit, v, fb_bit;
		
		shift_reg = 0; // initialize the shift register
		for (int i = RID_OFFSET, z = buffer.length - FOOTER_LENGTH; i < z; ++i) 
		{
		    v = buffer[i];
		    for (int j = 0; j < 8; ++j) // for each bit 
		    {
				data_bit = (byte) (v & 0x01); // isolate least sign bit
				sr_lsb = (byte) (shift_reg & 0x01);
				fb_bit = (byte) ((data_bit ^ sr_lsb) & 0x01); // calculate the feedback bit
				shift_reg = (byte) (shift_reg >>> 1);
				if (1 == fb_bit) {
				    shift_reg = (byte) (shift_reg ^ 0x8C);
				}
				v = (byte) (v >>> 1);
		    }
		}
		return (byte) shift_reg;
	}
	
	public static int ping(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put((byte) SETUP_COM).put((byte) dataLength);
		msg.put(PING);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	public static int ack(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put((byte) SETUP_COM).put((byte) dataLength);
		msg.put(ACK);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int motorSensorRequest(byte[] buffer, short packetNumber)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_MOTOR_SENSOR_DATA).put((byte) dataLength);
		msg.putShort(packetNumber);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int standardSensorRequest(byte[] buffer, short packetNumber)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_MOTOR_SENSOR_DATA).put((byte) dataLength); // header
		msg.putShort(packetNumber);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1); // footer
		return dataLength + METADATA_SIZE;
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
	public static int customSensorRequest(byte[] buffer, short packetNumber)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_CUSTOM_SENSOR_DATA).put((byte) dataLength);
		msg.putShort(packetNumber);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int allSensorRequest(byte[] buffer, short packetNumber)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_ALL_SENSOR_DATA).put((byte) dataLength);
		msg.putShort(packetNumber);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int enableMotorSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 0;
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_MOTOR_SENSOR_DATA).put((byte) dataLength);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int enableStandardSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 0;
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_STANDARD_SENSOR_DATA).put((byte) dataLength);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int enableCustomSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 0;
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_CUSTOM_SENSOR_DATA).put((byte) dataLength);
		msg.put((byte) checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int enableAllSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 0;
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_ALL_SENSOR_DATA).put((byte) dataLength);
		msg.put((byte) checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Disables batch updating of motor-related sensor packets.
	 * 
	 * @see enableMotorSensorSending
	 */
	public static int disableMotorSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_MOTOR_SENSOR_DATA).put((byte) dataLength);
		msg.put(DISABLE);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Disables batch updating of standard sensor packets.
	 * 
	 * @see enableStandardSensorSending
	 */
	public static int disableStandardSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_STANDARD_SENSOR_DATA).put((byte) dataLength);
		msg.put(DISABLE);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Disables batch updating of custom sensor packets.
	 * 
	 * @see enableCustomSensorSending
	 */
	public static int disableCustomSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_CUSTOM_SENSOR_DATA).put((byte) dataLength);
		msg.put(DISABLE);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Disables batch updating of all sensor packets.
	 * 
	 * @see enableAllSensorSending
	 */
	public static int disableAllSensorSending(byte[] buffer)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(GET_ALL_SENSOR_DATA).put((byte) dataLength);
		msg.put(DISABLE);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sets the refresh rate for batch updating by motor-related sensor packets.
	 * 
	 * @param milliseconds
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
	 * 
	 * @see motorSensorRequest
	 */
	public static int setMotorSensorPeriod(byte[] buffer, short milliseconds)
	{
		// TODO stub
		return 0;
	}
	
	/**
	 * Sets the refresh rate for batch updating by standard sensor packets.
	 * 
	 * @param milliseconds
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
	 * 
	 * @see standardSensorRequest
	 */
	public static int setStandardSensorPeriod(byte[] buffer, short milliseconds)
	{
		// TODO stub
		return 0;
	}
	
	/**
	 * Sets the refresh rate for batch updating by custom sensor packets.
	 * 
	 * @param milliseconds
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
	 * 
	 * @see customSensorRequest
	 */
	public static int setCustomSensorPeriod(byte[] buffer, short milliseconds)
	{
		// TODO stub
		return 0;
	}
	
	/**
	 * Sets the refresh rate for batch updating of all sensor packets.
	 * 
	 * @param milliseconds
	 *            The update period in milliseconds for batch sensing packets to
	 *            the PC central controller.
	 * 
	 *            Please note: The default timePeriod is 50ms. The maximum
	 *            refresh rate possible for the PMS5005 Sensing and Motion
	 *            Controller is 50ms @ 20Hz.
	 * 
	 * @see allSensorRequest
	 */
	public static int setAllSensorPeriod(byte[] buffer, short milliseconds)
	{
		// TODO stub
		return 0;
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
	public static int setCustomDOut(byte[] buffer, byte ival)
	{
		return 0;
	}
	
	/**
	 * The following is a description of this method from the WiRobot X80 User Manual pdf
	 * 
	 * sends two 16-bit words infrared communication output data to
	 * the Sensing and Motion Controller (PMS5005). The PMS5005 will then send the data out
	 * through the infrared Remote Controller Module (MIR5500). In the case of being used for
	 * infrared remote control, the output data serves as the remote control command.
	 * 
	 * Remarks:
	 * 1. In infrared communication application, the data format and the interpretation can
	 * 	be defined by the user at the application level.
	 * 2. In infrared remote control application, the control command should be compatible
	 * 	to the device to which the command is sent.
	 * 3. This API function is under development and will be available shortly.
	 *
	 * @param buffer command buffer to assign to this command
	 * @param lowWord 1st word
	 * @param highWord 2nd word
	 * @return
	 */
	public static int setInfraredControlOutput(byte[] buffer, int lowWord, int highWord)
	{
		return 0;
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
	public static int setMotorPolarity(byte[] buffer, byte channel, byte polarity)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 3*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PARAM_SET).put((byte) dataLength);
		msg.put(DC_SENSOR_USAGE); // Subcommand
		msg.put(channel);
		msg.put(polarity);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int enableDCMotor(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(ENABLE);
		msg.put(channel);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int disableDCMotor(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(DISABLE); // 0 = Disable/Suspend
		msg.put(channel); // 0=L | 1=R
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Resumes the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 */
	public static int resumeDCMotor(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(RESUME);
		msg.put(channel);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Suspends the specified DC motor control channel.
	 * 
	 * @param channel
	 *            0 for left, 1 for right (robot first person perspective)
	 * 
	 *            All motor control channels are initially suspended at boot-up.
	 */
	public static int suspendDCMotor(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(SUSPEND);
		msg.put(channel);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int setDCMotorPositionControlPID(byte[] buffer, byte channel, short kp, short kd, short ki)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 5*(Byte.SIZE >> 3) + 3*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PARAM_SET).put((byte) dataLength);
		msg.put(DC_POSITION_PID);
		msg.put(channel);
		msg.put(PROPORTIONAL_GAIN);
		msg.putShort(kp);
		msg.put(DERIVATIVE_GAIN);
		msg.putShort(kd);
		msg.put(INTEGRAL_GAIN);
		msg.putShort(ki);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	public static int setDCMotorVelocityControlPID(byte[] buffer, byte channel, short kp, short kd, short ki)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 5*(Byte.SIZE >> 3) + 3*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PARAM_SET).put((byte) dataLength);
		msg.put(DC_VELOCITY_PID);
		msg.put(channel);
		msg.put(PROPORTIONAL_GAIN);
		msg.putShort(kp);
		msg.put(DERIVATIVE_GAIN);
		msg.putShort(kd);
		msg.put(INTEGRAL_GAIN);
		msg.putShort(ki);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * This filtering feature is still under development. All data will be
	 * treated as raw data.
	 */
	public static int setDCMotorSensorFilter(byte[] buffer, byte channel, short filterMethod)
	{
		return 0;
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
	public static int setDCMotorSensorUsage(byte[] buffer, byte channel, byte sensorType)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 3*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PARAM_SET).put((byte) dataLength);
		msg.put(DC_SENSOR_USAGE); // Subcommand
		msg.put(channel); // 0-5 = Single Potentiometer, 0-2 = Dual Potentiometer, 0-1 = Encoder
		msg.put(sensorType); // 0x00 = Single Potentiometer, 0x01 = Dual Potentiometer, 0x02 = Encoder
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int setDCMotorControlMode(byte[] buffer, byte channel, byte controlMode)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 3*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PARAM_SET).put((byte) dataLength);
		msg.put(DC_CTRL_MODE); // Subcommand
		msg.put(channel); // channel 0-5
		msg.put(controlMode); // 0 = open, 1 = closed position, 2 = closed // velocity
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	 * @param position
	 *            Target position value
	 * @param milliseconds
	 *            Executing time in milliseconds
	 */
	public static int setDCMotorPosition(byte[] buffer, byte channel, short position, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3) + 2*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(POSITION_CTRL).put((byte) dataLength);
		msg.put(channel); // Channel 0-5
		msg.putShort(position);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	 * @param position
	 *            Target position value
	 * 
	 *            1) Motor will be enabled automatically by the system when this
	 *            command is received. 2) No velocity is available for motor
	 *            channel using single potentiometer sensor. 3) The unit of the
	 *            velocity is (Position change in A/D sampling data) / second
	 *            when using dual potentiometer sensor for rotational position
	 *            measurement and pulse/second when using quadrature encoder.
	 * 
	 * @see DCMotorVelocityTimeCtrl
	 * @see getSensorPot
	 */
	public static int setDCMotorPosition(byte[] buffer, byte channel, short position)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 3*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(POSITION_CTRL).put((byte) dataLength);
		msg.put(channel); // channel 0-5
		msg.putShort(position);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	 * @param milliseconds
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
	public static int setDCMotorPulse(byte[] buffer, byte channel, short pulseWidth, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3) + 2*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PWM_CTRL).put((byte) dataLength);
		msg.put(channel); // Channel 0-5
		msg.putShort(pulseWidth);
		msg.put(SINGLE_CHANNEL_TIMED_FLAG);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int setDCMotorPulse(byte[] buffer, byte channel, short pulseWidth)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3) + (Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(PWM_CTRL).put((byte) dataLength);
		msg.put(channel); // Channel 0-5
		msg.putShort(pulseWidth);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the position control command to all 6 DC motor control channels on
	 * the sensing and motion controller (PMS5005) at the same time. The command
	 * includes the target positions and the time period to execute the command.
	 * The current trajectory planning method for time control is linear.
	 * 
	 * @param position0
	 *            Target position for channel #1
	 * @param position1
	 *            Target position for channel #2
	 * @param position2
	 *            Target position for channel #3
	 * @param position3
	 *            Target position for channel #4
	 * @param position4
	 *            Target position for channel #5
	 * @param position5
	 *            Target position for channel #6
	 * @param milliseconds
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
	public static int setAllDCMotorPositions(byte[] buffer, short position0, short position1, 
			short position2, short position3, short position4, short position5, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 7*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_POSITION_CTRL).put((byte) dataLength);
		msg.putShort(position0);
		msg.putShort(position1);
		msg.putShort(position2);
		msg.putShort(position3);
		msg.putShort(position4);
		msg.putShort(position5);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the position control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target positions without a specified time period for
	 * execution. The motion controller will drive the motor to reach the target
	 * position with maximum effort.
	 * 
	 * @param position0
	 *            Target position for channel #1
	 * @param position1
	 *            Target position for channel #2
	 * @param position2
	 *            Target position for channel #3
	 * @param position3
	 *            Target position for channel #4
	 * @param position4
	 *            Target position for channel #5
	 * @param position5
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
	public static int setAllDCMotorPositions(byte[] buffer, short position0, short position1, 
			short position2, short position3, short position4, short position5)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 6*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_POSITION_CTRL).put((byte) dataLength);
		msg.putShort(position0);
		msg.putShort(position1);
		msg.putShort(position2);
		msg.putShort(position3);
		msg.putShort(position4);
		msg.putShort(position5);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the velocity control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target velocities and the time period to execute the
	 * command. The trajectory planning method for time control is linear.
	 * 
	 * @param velocity0
	 *            Target velocity for channel #1
	 * @param velocity1
	 *            Target velocity for channel #2
	 * @param velocity2
	 *            Target velocity for channel #3
	 * @param velocity3
	 *            Target velocity for channel #4
	 * @param velocity4
	 *            Target velocity for channel #5
	 * @param velocity5
	 *            Target velocity for channel #6
	 * @param milliseconds
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
	public static int setAllDCMotorVelocities(byte[] buffer, short velocity0, short velocity1, 
			short velocity2, short velocity3, short velocity4, short velocity5, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 7*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_VELOCITY_CTRL).put((byte) dataLength);
		msg.putShort(velocity0);
		msg.putShort(velocity1);
		msg.putShort(velocity2);
		msg.putShort(velocity3);
		msg.putShort(velocity4);
		msg.putShort(velocity5);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the velocity control command to all 6 DC motor control channels on
	 * the Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target velocities without specifying an execution time
	 * period. The motion controller will drive the motor to achieve the target
	 * velocity with maximum effort.
	 * 
	 * @param velocity0
	 *            Target velocity for channel #1
	 * @param velocity1
	 *            Target velocity for channel #2
	 * @param velocity2
	 *            Target velocity for channel #3
	 * @param velocity3
	 *            Target velocity for channel #4
	 * @param velocity4
	 *            Target velocity for channel #5
	 * @param velocity5
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
	public static int setAllDCMotorVelocities(byte[] buffer, short velocity0, short velocity1, 
			short velocity2, short velocity3, short velocity4, short velocity5)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 6*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_VELOCITY_CTRL).put((byte) dataLength);
		msg.putShort(velocity0);
		msg.putShort(velocity1);
		msg.putShort(velocity2);
		msg.putShort(velocity3);
		msg.putShort(velocity4);
		msg.putShort(velocity5);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the PWM control command to all 6 DC motor control channels on the
	 * Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target PWM values and the time period for execution. The
	 * current trajectory planning method for time control is linear.
	 * 
	 * @param pulseWidth0
	 *            Target PWM value (pulse width) for channel #1
	 * @param pulseWidth1
	 *            Target PWM value (pulse width) for channel #2
	 * @param pulseWidth2
	 *            Target PWM value (pulse width) for channel #3
	 * @param pulseWidth3
	 *            Target PWM value (pulse width) for channel #4
	 * @param pulseWidth4
	 *            Target PWM value (pulse width) for channel #5
	 * @param pulseWidth5
	 *            Target PWM value (pulse width) for channel #6
	 * @param milliseconds
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
	public static int setAllDCMotorPulses(byte[] buffer, short pulseWidth0, short pulseWidth1, 
			short pulseWidth2, short pulseWidth3, short pulseWidth4, short pulseWidth5, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 7*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_PWM_CTRL).put((byte) dataLength);
		msg.putShort(pulseWidth0);
		msg.putShort(pulseWidth1);
		msg.putShort(pulseWidth2);
		msg.putShort(pulseWidth3);
		msg.putShort(pulseWidth4);
		msg.putShort(pulseWidth5);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the PWM control command to all 6 DC motor control channels on the
	 * Sensing and Motion Controller (PMS5005) at the same time. The command
	 * includes the target PWM values without a specified time period for
	 * execution. The motion controller will adjust the pulse width right away.
	 * 
	 * @param pulseWidth0
	 *            Target PWM value (pulse width) for channel #1
	 * @param pulseWidth1
	 *            Target PWM value (pulse width) for channel #2
	 * @param pulseWidth2
	 *            Target PWM value (pulse width) for channel #3
	 * @param pulseWidth3
	 *            Target PWM value (pulse width) for channel #4
	 * @param pulseWidth4
	 *            Target PWM value (pulse width) for channel #5
	 * @param pulseWidth5
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
	public static int setAllDCMotorPulses(byte[] buffer, short pulseWidth0, short pulseWidth1, 
			short pulseWidth2, short pulseWidth3, short pulseWidth4, short pulseWidth5)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 6*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_PWM_CTRL).put((byte) dataLength);
		msg.putShort(pulseWidth0);
		msg.putShort(pulseWidth1);
		msg.putShort(pulseWidth2);
		msg.putShort(pulseWidth3);
		msg.putShort(pulseWidth4);
		msg.putShort(pulseWidth5);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Enables the specified servo motor control channel.
	 * 
	 * @param channel
	 *            0, 1, 2, 3, 4, or 5
	 * 
	 *            All servo motor channels are disabled initially at system
	 *            startup. They need to be enabled explicitly before use.
	 * 
	 * @see disableServo
	 */
	public static int enableServo(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(ENABLE); // 1 = Enable
		msg.put(channel); // 6-11 SERVO
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int disableServo(byte[] buffer, byte channel)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(TOGGLE_DC_MOTORS).put((byte) dataLength);
		msg.put(DISABLE); // 0 = Disable
		msg.put(channel);  // 6-11 = SERVO
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	 * @param milliseconds
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
	public static int setServoPulse(byte[] buffer, byte channel, short pulseWidth, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3) + 2*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(SERVO_CTRL).put((byte) dataLength);
		msg.put(channel);
		msg.putShort(pulseWidth);
		msg.put(SINGLE_CHANNEL_TIMED_FLAG);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int setServoPulse(byte[] buffer, byte channel, short pulseWidth)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 2*(Byte.SIZE >> 3) + (Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(SERVO_CTRL).put((byte) dataLength);
		msg.put(channel);
		msg.putShort(pulseWidth);
		msg.put(SINGLE_CHANNEL_TIMED_FLAG);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends PWM control commands to all 6 servo motor control channels
	 * on the Sensing and Motion Controller (PMS5005) at the same time.
	 * 
	 * The command includes the target position commands and the time period to
	 * execute the command. The current trajectory planning method for time
	 * control is linear.
	 * 
	 * @param pulseWidth0
	 *            Target pulse width for channel #1 (Left Motor on X80Pro)
	 * @param pulseWidth1
	 *            Target pulse width for channel #2 (-Right Motor on X80Pro)
	 * @param pulseWidth2
	 *            Target pulse width for channel #3 (NO_CTRL on X80Pro)
	 * @param pulseWidth3
	 *            Target pulse width for channel #4 (NO_CTRL on X80Pro)
	 * @param pulseWidth4
	 *            Target pulse width for channel #5 (NO_CTRL on X80Pro)
	 * @param pulseWidth5
	 *            Target pulse width for channel #6 (NO_CTRL on X80Pro)
	 * @param milliseconds
	 *            Executing time in milliseconds
	 * 
	 *            When omitting servo motors from control, please send the
	 *            command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 */
	public static int setAllServoPulses(byte[] buffer, short pulseWidth0, short pulseWidth1, 
			short pulseWidth2, short pulseWidth3, short pulseWidth4, short pulseWidth5, short milliseconds)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 7*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_SERVO_CTRL).put((byte) dataLength);
		msg.putShort(pulseWidth0);
		msg.putShort(pulseWidth1);
		msg.putShort(pulseWidth2);
		msg.putShort(pulseWidth3);
		msg.putShort(pulseWidth4);
		msg.putShort(pulseWidth5);
		msg.putShort(milliseconds);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
	
	/**
	 * Sends the PWM control command to all 6 servo motor control channels
	 * on the Sensing and Motion Controller (PMS5005) at the same time.
	 * 
	 * The command includes the target position commands without specifying a
	 * time period in which to execute the command. The motion controller sends
	 * the desired pulse width to the servo motor right away.
	 * 
	 * @param pulseWidth0
	 *            Target pulse width for channel #1 (Left Motor on X80Pro)
	 * @param pulseWidth1
	 *            Target pulse width for channel #2 (-Right Motor on X80Pro)
	 * @param pulseWidth2
	 *            Target pulse width for channel #3 (NO_CTRL on X80Pro)
	 * @param pulseWidth3
	 *            Target pulse width for channel #4 (NO_CTRL on X80Pro)
	 * @param pulseWidth4
	 *            Target pulse width for channel #5 (NO_CTRL on X80Pro)
	 * @param pulseWidth5
	 *            Target pulse width for channel #6 (NO_CTRL on X80Pro)
	 * 
	 *            When omitting servo motors from control, please send the
	 *            command value -32768 (0x8000), which implies NO_CTRL.
	 * 
	 * @see servoNonTimeCtrl
	 */
	public static int setAllServoPulses(byte[] buffer, short pulseWidth0, short pulseWidth1, 
			short pulseWidth2, short pulseWidth3, short pulseWidth4, short pulseWidth5)
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = 6*(Short.SIZE >> 3);
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(ALL_SERVO_CTRL).put((byte) dataLength);
		msg.putShort(pulseWidth0);
		msg.putShort(pulseWidth1);
		msg.putShort(pulseWidth2);
		msg.putShort(pulseWidth3);
		msg.putShort(pulseWidth4);
		msg.putShort(pulseWidth5);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
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
	public static int setLCDDisplayPMS(byte[] buffer, byte[] frame, byte frameNumber) // LCD PMS
	{
		ByteBuffer msg = ByteBuffer.wrap(buffer);
		msg.order(PMS5005.BYTE_ORDER);
		int dataLength = (Byte.SIZE >> 3) + FRAME_LENGTH*(Byte.SIZE >> 3); // 1 frame slice byte + 64 bytes of bitmap data)
		msg.put(STX0).put(STX1).put(RID_PMS5005).put(RESERVED).put(LCD_CTRL).put((byte) dataLength);
		msg.put(frameNumber);
		for (int i = 0; i < FRAME_LENGTH; ++i) msg.put(frame[i]);
		msg.put(checksum(msg.array())).put(ETX0).put(ETX1);
		return dataLength + METADATA_SIZE;
	}
}
