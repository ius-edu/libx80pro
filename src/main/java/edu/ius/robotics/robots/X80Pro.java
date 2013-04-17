/* 
 * X80Pro.java written by Jesse Riddle, with significant 
 * documentation taken from the DrRobot WiRobot SDK Application Programming 
 * Interface (API) Reference Manual - (For MS Windows) Version: 1.0.8 Feb. 2004 
 * by DrRobot, Inc. and some parts from the DrRobot Java motion control demo 
 * program.
 */

package edu.ius.robotics.robots;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.lang.Runtime;
import java.nio.ByteBuffer;

import javax.imageio.ImageIO;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.boards.PMB5010;
import edu.ius.robotics.robots.codecs.ADPCM;
import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;

import edu.ius.robotics.robots.sockets.UDPSocket;
import edu.ius.robotics.robots.worker.Worker;

public class X80Pro implements IRobot, Runnable
{
	public static final int DEFAULT_ROBOT_PORT = 10001;

	private static final int TX_BUFFER_SIZE = 1024;
	private static final int RX_BUFFER_SIZE = 8192;
	
	public static final byte L = 0;
	public static final byte R = 1;
	
	public static final byte X = 0;
	public static final byte Y = 1;
	
	public static class Sensors
	{
		public static final byte USAGE_SINGLE_POTENTIOMETER = 0x00;		/* Single */
		public static final byte USAGE_DUAL_POTENTIOMETER = 0x01;		/* Dual */
		public static final byte USAGE_ENCODER = 0x02;					/* Quadrature */
		
		public static final byte TYPE_SONAR = PMS5005.GET_STANDARD_SENSOR_DATA;
		public static final byte TYPE_IR_FRONT_LEFT = PMS5005.GET_STANDARD_SENSOR_DATA;
		public static final byte TYPE_IR_NON_FRONT_LEFT = PMS5005.GET_CUSTOM_SENSOR_DATA;
		public static final byte TYPE_DC_MOTOR = PMS5005.GET_MOTOR_SENSOR_DATA;
		public static final byte TYPE_SERVO_MOTOR = PMS5005.GET_MOTOR_SENSOR_DATA;
		
		public static final byte TYPE_STANDARD_GENERAL = PMS5005.GET_STANDARD_SENSOR_DATA;
		public static final byte TYPE_CUSTOM_GENERAL = PMS5005.GET_CUSTOM_SENSOR_DATA;
		public static final byte TYPE_MOTOR_GENERAL = PMS5005.GET_MOTOR_SENSOR_DATA;
		
		public static final double IR_DISTANCE_MAXIMUM = 0.81;
		public static final double IR_DISTANCE_MINIMUM = 0.09;
		
		public static final double SONAR_DISTANCE_MAXIMUM = 1.01;
		public static final double SONAR_DISTANCE_MINIMUM = 0.05;
		
		// Standard and Custom sensors
		public static final int NUM_STANDARD_SENSORS = 6;
		public static final int NUM_DC_MOTOR_CHANNELS = 2;
		public static final int NUM_INFRARED_SENSORS = 8;
		public static final int NUM_INFRARED_SENSORS_FRONT = 4;
		public static final int NUM_CUSTOM_AD_SENSORS = 8;
		public static final int NUM_SONAR_SENSORS = 6;
		public static final int NUM_SONAR_SENSORS_FRONT = 3;
		public static final int NUM_HUMAN_SENSORS = 2;
		public static final int NUM_IO_PORT_SENSORS = 8;
		public static final int NUM_LEFT_CONSTELLATION_SENSORS = 4;
		public static final int NUM_RIGHT_CONSTELLATION_SENSORS = 4;
		public static final int NUM_RELATIVE_CONSTELLATION_SENSORS = 4;
		public static final int NUM_TILTING_SENSORS = 2;
		public static final int NUM_OVERHEAT_SENSORS = 2;
		public static final int NUM_TEMPERATURE_SENSORS = 2;
		public static final int NUM_INFRARED_RECEIVERS = 4;
		
		// Motor Sensors
		public static final int NUM_POTENTIOMETER_AD_SENSORS = 6;
		public static final int NUM_MOTOR_CURRENT_AD_SENSORS = 6;
		public static final int NUM_ENCODER_PULSE_SENSORS = 2;
		public static final int NUM_ENCODER_SPEED_SENSORS = 2;
		public static final int NUM_ENCODER_DIRECTION_SENSORS = 2;
		
		public static final int C_FULL_COUNT = 32767;
		public static final int C_WHOLE_RANGE = 1200;
		
		public static double[] INFRARED_SENSOR_POSITION;
		public static double[] SONAR_SENSOR_POSITION;
		
		public static int FRONT_FAR_LEFT_INFRARED_SENSOR = 0;
		public static int FRONT_MID_LEFT_INFRARED_SENSOR_INDEX = 1;
		public static int FRONT_MID_RIGHT_INFRARED_SENSOR_INDEX = 2;
		public static int FRONT_FAR_RIGHT_INFRARED_SENSOR = 3;
		
		public static int FRONT_LEFT_SONAR_SENSOR = 0;
		public static int FRONT_MIDDLE_SONAR_SENSOR = 1;
		public static int FRONT_RIGHT_SONAR_SENSOR = 2;
		
		public static final int ACCEPTABLE_ENCODER_DEVIATION = 16;
		
		static 
		{
			INFRARED_SENSOR_POSITION = new double[4];
			INFRARED_SENSOR_POSITION[0] = 0.785;
			INFRARED_SENSOR_POSITION[1] = 0.35;
			INFRARED_SENSOR_POSITION[2] = -0.35;
			INFRARED_SENSOR_POSITION[3] = -0.785;
			
			SONAR_SENSOR_POSITION = new double[3];
			SONAR_SENSOR_POSITION[0] = 0.8;
			SONAR_SENSOR_POSITION[1] = 0;
			SONAR_SENSOR_POSITION[2] = -0.8;
		}
	}
	
	public static class Motors
	{
		public static final short NO_CTRL = -32768;
		public static final short MAX_SPEED = 32767;
		
		public static final byte CONTROL_MODE_PWM = 0;
		public static final byte CONTROL_MODE_POSITION = 1;
		public static final byte CONTROL_MODE_VELOCITY = 2;
		
		public static final byte POLARITY_POSITIVE = 1;
		public static final byte POLARITY_NEGATIVE = -1;
		
		public static final short SERVO_Y_INI = 3650;
		public static final short SERVO_X_INI = 3300;
		
		public static final byte DIRECTION = -1;
		public static final short MAX_VELOCITY = 1200;
		public static final short VELOCITY = 1000;
		
		public static final short PWM_N = 16383;
		public static final short PWM_O = 8000;
		public static final short MAX_PWM_L = 32767;
		public static final short MAX_PWM_R = 0;
		public static final short DUTY_CYCLE_UNIT = 8383;
	}
	
	public static class Pkg
	{
		public static final int STX0_OFFSET = 0;
		public static final int STX1_OFFSET = 1;
		public static final int RID_OFFSET = 2;
		public static final int SEQ_OFFSET = 3;
		public static final int DID_OFFSET = 4;
		public static final int LENGTH_OFFSET = 5;
		public static final int DATA_OFFSET = 6;
		public static final int CHECKSUM_RELATIVE_OFFSET = 0;
		public static final int ETX0_RELATIVE_OFFSET = 1;
		public static final int ETX1_RELATIVE_OFFSET = 2;
		public static final int HEADER_LENGTH = 6;
		public static final int FOOTER_LENGTH = 3;
		public static final int METADATA_SIZE = 9;
		public static final int IMAGE_PKG_COUNT_RELATIVE_OFFSET = 1;
		public static final int IMAGE_DATA_RELATIVE_OFFSET = 2;
		
		public static final int MAX_DATA_SIZE = 0xFF;
		public static final int STX0 = 0x5E;
		public static final int STX1 = 0x02;
		public static final int ETX0 = 0x5E;
		public static final int ETX1 = 0x0D;
		
		int offset;
		int length;
		public ByteBuffer data;
		public ByteBuffer msg;
		
		public String robotIP;
		public int robotPort;
		
		public void clear()
		{
			robotIP = "";
			offset = length = robotPort = 0;
			data.clear();
			msg.clear();
		}
		
		public Pkg()
		{
			data = ByteBuffer.allocate(MAX_DATA_SIZE);
			msg = ByteBuffer.allocate(MAX_DATA_SIZE + METADATA_SIZE);
			clear();
		}
		
		// This is from PMS5005 and PMB5010 protocol documentation
		public byte checksum() // could, maybe should, be static (checksum(byte[] buf))
		{
			int shift_reg, sr_lsb, data_bit, v;
			int fb_bit;
			int z;
			shift_reg = 0; // initialize the shift register
			z = length - 5;
			for (int i = 0; i < z; ++i) 
			{
			    v = (msg.get(2 + i) & 0xFF); // start from RID
			    // for each bit
			    for (int j = 0; j < 8; ++j) 
			    {
					// isolate least sign bit
					data_bit = ((v & 0x01) & 0xFF);
					sr_lsb = ((shift_reg & 0x01) & 0xFF);
					// calculate the feed back bit
					fb_bit = (((data_bit ^ sr_lsb) & 0x01) & 0xFF);
					shift_reg = ((shift_reg & 0xFF) >> 1);
					if (fb_bit == 1)
					{
					    shift_reg = ((shift_reg ^ 0x8C) & 0xFF);
					}
					v = ((v & 0xFF) >> 1);
			    }
			}
			return (byte) shift_reg;
		}
		
		public void print()
		{
			System.out.println("length: " + length);
			System.out.print("data: ");
			for (int i = 0; i < MAX_DATA_SIZE; ++i)
			{
				System.out.print(data.get(i) + " ");
			}
			System.out.println();
			System.out.println("robotIP: " + robotIP);
			System.out.println("robotPort: " + robotPort);
		}
	}
	
	private class MotorSensorData
	{
		public int[] potentiometerAD;
		public int[] motorCurrentAD;
		public int[] encoderPulse;
		public int[] encoderSpeed;
		public int[] encoderDirection;
		
		public MotorSensorData()
		{
			potentiometerAD = new int[Sensors.NUM_POTENTIOMETER_AD_SENSORS];
			motorCurrentAD = new int[Sensors.NUM_MOTOR_CURRENT_AD_SENSORS];
			encoderPulse = new int[Sensors.NUM_ENCODER_PULSE_SENSORS];
			encoderSpeed = new int[Sensors.NUM_ENCODER_SPEED_SENSORS];
			encoderDirection = new int[Sensors.NUM_ENCODER_DIRECTION_SENSORS];
		}
		
		public void setMotorSensorData(ByteBuffer motorSensorData)
		{
			int offset = 0, i;
			
			for (i = 0; i < Sensors.NUM_POTENTIOMETER_AD_SENSORS; ++i) 
			{
				potentiometerAD[i] = motorSensorData.getShort(offset + 2*i) & 0xFFFF;
			}
			offset += Sensors.NUM_POTENTIOMETER_AD_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_MOTOR_CURRENT_AD_SENSORS; ++i) 
			{
				motorCurrentAD[i] = motorSensorData.getShort(offset + 2*i) & 0xFFFF;
			}
			offset += Sensors.NUM_MOTOR_CURRENT_AD_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_ENCODER_PULSE_SENSORS; ++i) 
			{
				encoderPulse[i] = motorSensorData.getShort(offset + 4*i) & 0xFFFF;
			}
			offset += (Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_ENCODER_SPEED_SENSORS; ++i) 
			{
				encoderSpeed[i] = motorSensorData.getShort(offset + 4*i) & 0xFFFF;
			}
			offset += 3*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_ENCODER_DIRECTION_SENSORS; ++i) 
			{
				encoderDirection[i] = motorSensorData.get(offset) & (i+1);
			}
		}
	}
	
	private class CustomSensorData
	{
		public int[] customAD;
		
		public int[] ioPort;
		public int[] mmDistanceToLeftConstellation;
		public int[] mmDistanceToRightConstellation;
		public int[] mmDistanceToRelativeConstellation;
		
		public CustomSensorData()
		{
			customAD = new int[Sensors.NUM_CUSTOM_AD_SENSORS];
			ioPort = new int[Sensors.NUM_IO_PORT_SENSORS];
			mmDistanceToLeftConstellation = new int[Sensors.NUM_LEFT_CONSTELLATION_SENSORS];
			mmDistanceToRightConstellation = new int[Sensors.NUM_RIGHT_CONSTELLATION_SENSORS];
			mmDistanceToRelativeConstellation = new int[Sensors.NUM_RELATIVE_CONSTELLATION_SENSORS];
		}
		
		public void setCustomSensorData(ByteBuffer customSensorData)
		{
			int offset = 0, i;
			
			for (i = 0; i < Sensors.NUM_CUSTOM_AD_SENSORS; ++i)
			{
				customAD[i] = customSensorData.getShort(offset + 2*i) & 0xFFFF;
			}
			offset += Sensors.NUM_CUSTOM_AD_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_IO_PORT_SENSORS; ++i)
			{
				ioPort[i] = (byte) customSensorData.get(offset) & (0x01 << i);
			}
			offset += (Sensors.NUM_IO_PORT_SENSORS >> 3)*(Byte.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_LEFT_CONSTELLATION_SENSORS; ++i)
			{
				mmDistanceToLeftConstellation[i] = customSensorData.getShort(offset + 2*i) & 0xFFFF;
			}
			offset += Sensors.NUM_LEFT_CONSTELLATION_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_RIGHT_CONSTELLATION_SENSORS; ++i)
			{
				mmDistanceToRightConstellation[i] = customSensorData.getShort(offset + 2*i) & 0xFFFF;
			}
			offset += Sensors.NUM_RIGHT_CONSTELLATION_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_RELATIVE_CONSTELLATION_SENSORS; ++i)
			{
				mmDistanceToRelativeConstellation[i] = customSensorData.getShort(offset + 2*i) & 0xFFFF; 				
			}
		}
	}
	
	private class StandardSensorData
	{
		public int[] sonarDistance;
		public int[] humanAlarm;
		public int[] motionDetect;
		public int[] tiltingAD;
		public int[] overheatAD;
		public int temperatureAD;
		public int infraredRangeAD;
		public int[] infraredCommand;
		public int mainboardBatteryVoltageAD_0to9V;
		public int motorBatteryVoltageAD_0to24V;
		public int servoBatteryVoltageAD_0to9V;
		public int referenceVoltageAD_Vcc;
		public int potentiometerVoltageAD_Vref;
		
		public StandardSensorData()
		{
			sonarDistance = new int[Sensors.NUM_SONAR_SENSORS];
			humanAlarm = new int[Sensors.NUM_HUMAN_SENSORS];
			motionDetect = new int[Sensors.NUM_HUMAN_SENSORS];
			tiltingAD = new int[Sensors.NUM_TILTING_SENSORS];
			overheatAD = new int[Sensors.NUM_TEMPERATURE_SENSORS];
			temperatureAD = -1;
			infraredRangeAD = -1;
			infraredCommand = new int[Sensors.NUM_INFRARED_RECEIVERS];
			mainboardBatteryVoltageAD_0to9V = -1;
			motorBatteryVoltageAD_0to24V = -1;
			servoBatteryVoltageAD_0to9V = -1;
			referenceVoltageAD_Vcc = -1;
			potentiometerVoltageAD_Vref = -1;
		}
		
		public void setStandardSensorData(ByteBuffer standardSensorData)
		{
			int offset = 0, i;
			for (i = 0; i < Sensors.NUM_SONAR_SENSORS; ++i)
			{
				sonarDistance[i] = standardSensorData.get(offset + i) & 0xFF;
			}
			offset += Sensors.NUM_SONAR_SENSORS*(Byte.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_HUMAN_SENSORS; ++i)
			{
				humanAlarm[i] = standardSensorData.getShort(offset + 2*(Short.SIZE >> 3)*i) & 0x7FFF;
			}
			offset += Short.SIZE >> 3;
			
			for (i = 0; i < Sensors.NUM_HUMAN_SENSORS; ++i)
			{
				motionDetect[i] = standardSensorData.getShort(offset + 2*(Short.SIZE >> 3)*i) & 0x7FFF;
			}
			offset += 3*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_TILTING_SENSORS; ++i)
			{
				tiltingAD[i] = standardSensorData.getShort(offset + (Short.SIZE >> 3)*i) & 0x7FFF;
			}
			offset += Sensors.NUM_TILTING_SENSORS*(Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_OVERHEAT_SENSORS; ++i)
			{
				overheatAD[i] = standardSensorData.getShort(offset + (Short.SIZE >> 3)*i) & 0x7FFF;
			}
			offset += Sensors.NUM_OVERHEAT_SENSORS*(Short.SIZE >> 3);
			
			temperatureAD = standardSensorData.getShort(offset + (Short.SIZE >> 3)*i) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			infraredRangeAD = standardSensorData.getShort(offset) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			for (i = 0; i < Sensors.NUM_INFRARED_RECEIVERS; ++i)
			{
				infraredCommand[i] = standardSensorData.get(offset + i) & 0xFF; 
			}
			offset += Sensors.NUM_INFRARED_RECEIVERS*(Byte.SIZE >> 3);
			
			mainboardBatteryVoltageAD_0to9V = standardSensorData.getShort(offset) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			motorBatteryVoltageAD_0to24V = standardSensorData.getShort(offset) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			servoBatteryVoltageAD_0to9V = standardSensorData.getShort(offset) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			referenceVoltageAD_Vcc = standardSensorData.getShort(offset) & 0x7FFF;
			offset += (Short.SIZE >> 3);
			
			potentiometerVoltageAD_Vref = standardSensorData.getShort(offset) & 0x7FFF;
		}
	}
	
	/** wheel distance */
	public static final short WHEEL_CIRCUMFERENCE_ENCODER_COUNT = 800; // encoder units
	public static final double WHEEL_CIRCUMFERENCE = 0.265; // meters
	public static final double WHEEL_DISPLACEMENT = 0.297; // meters (default: 0.305) // (old: 0.2897)
	
	/** wheel radius */
	public static final double WHEEL_RADIUS = 0.085; // ~0.0825-0.085 meters
	
	/** robot state */
	private int[] encoderPulseInitial;
	private double heading;
	
	private MotorSensorData motorSensorData;
	private CustomSensorData customSensorData;
	private StandardSensorData standardSensorData;
	//private int[] powerSensorData;
	
	//private boolean[] lockIRRange;
	private Pkg pkg;
	private UDPSocket socket;
	private ADPCM adpcm;
	private ByteArrayOutputStream imageBuffer;
	private IRobotEventHandler iRobotEventHandler;
	private int previousSEQ;
	
	private boolean busyFlag;
	private int busyProgress;
	private Worker worker;
	
	private byte[] txBuffer; // sending
	private byte[] rxBuffer; // receiving
	int txBufferLength;
	int rxBufferLength;
	
	private void preInit()
	{
		pkg = new Pkg();
		
		txBuffer = new byte[TX_BUFFER_SIZE];
		rxBuffer = new byte[RX_BUFFER_SIZE];
		
		iRobotEventHandler = null;
		adpcm = new ADPCM();
		imageBuffer = new ByteArrayOutputStream();
		
		motorSensorData = new MotorSensorData();
		customSensorData = new CustomSensorData();
		standardSensorData = new StandardSensorData();

		// encoderPulseInitial is used as an independent variable to determine the robot's heading in radians.
		// encoderPulseInitial may be reset at any time by calling resetHeading();
		encoderPulseInitial = new int[Sensors.NUM_DC_MOTOR_CHANNELS];
	}
	
	private void postInit(boolean shouldResumeSensorSending)
	{
		if (shouldResumeSensorSending)
		{
			txBufferLength = PMS5005.enableMotorSensorSending(txBuffer); 
			socket.send(txBuffer, txBufferLength);
			txBufferLength = PMS5005.enableCustomSensorSending(txBuffer); 
			socket.send(txBuffer, txBufferLength);
			txBufferLength = PMS5005.enableStandardSensorSending(txBuffer);
			socket.send(txBuffer, txBufferLength);
			
			encoderPulseInitial[L] = absEncoderPulseValue(motorSensorData.encoderPulse[L]);
			encoderPulseInitial[R] = absEncoderPulseValue(motorSensorData.encoderPulse[R]);
		}
		else  
		{
			txBufferLength = PMS5005.disableMotorSensorSending(txBuffer);
			socket.send(txBuffer, txBufferLength);
			PMS5005.disableCustomSensorSending(txBuffer);
			socket.send(txBuffer, txBufferLength);
			PMS5005.disableStandardSensorSending(txBuffer);
			socket.send(txBuffer, txBufferLength);
			
			encoderPulseInitial[L] = -1;
			encoderPulseInitial[R] = -1;
		}
		
		txBufferLength = PMS5005.setMotorPolarity(txBuffer, L, Motors.POLARITY_POSITIVE);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setMotorPolarity(txBuffer, R, Motors.POLARITY_NEGATIVE);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, L, X80Pro.Sensors.USAGE_ENCODER);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, R, X80Pro.Sensors.USAGE_ENCODER);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, X80Pro.Motors.CONTROL_MODE_PWM);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, X80Pro.Motors.CONTROL_MODE_PWM);
		socket.send(txBuffer, txBufferLength);		
		
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, Motors.SERVO_Y_INI, Motors.SERVO_X_INI, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
		
		// This shutdown hook indicates other threads should terminate as well (e.g. sensor data collection thread).
		attachShutdownHook();
	}
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param ipAddress
	 */
	public X80Pro(String ipAddress) throws IOException
	{
		preInit();
		socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		postInit(true);
	}
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param robotIp
	 * @param robotPort
	 */
	public X80Pro(String ipAddress, int port) throws IOException
	{
		preInit();
		socket = new UDPSocket(this, ipAddress, port);
		postInit(true);
	}
	
	public X80Pro(String ipAddress, IRobotEventHandler iRobotEventHandler) throws IOException
	{
		preInit();
		this.iRobotEventHandler = iRobotEventHandler;
		socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		postInit(true);
	}
	
	public X80Pro(String ipAddress, IRobotEventHandler iRobotEventHandler, boolean shouldResumeSensorSending) throws IOException
	{
		preInit();
		this.iRobotEventHandler = iRobotEventHandler;
		socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		postInit(shouldResumeSensorSending);
	}
	
	public X80Pro(String ipAddress, int port, IRobotEventHandler iRobotEventHandler) throws IOException
	{
		preInit();
		this.iRobotEventHandler = iRobotEventHandler;
		socket = new UDPSocket(this, ipAddress, port);
		postInit(true);
	}
	
	public X80Pro(String ipAddress, int port, IRobotEventHandler iRobotEventHandler, boolean shouldResumeSensorSending) throws IOException
	{
		preInit();
		this.iRobotEventHandler = iRobotEventHandler;
		socket = new UDPSocket(this, ipAddress, port);
		postInit(shouldResumeSensorSending);
	}
	
	/**
	 * sends a custom command to the robot
	 * it's crazy to support this...	                                                                                                                              ... or is it?
	 * @param command
	 */
	public void sendCommand(ByteBuffer data, int dataLength)
	{
		socket.send(data.array(), dataLength);
	}
	
	private void dispatch(Pkg pkg)
	{
		// These two offsets are currently the same, but the actual sources are different. 
		// Could change with a firmware update?
		int pms5005MessageType = pkg.msg.get(PMS5005.DID_OFFSET) & 0xFF;
		int pmb5010MessageType = pkg.msg.get(PMB5010.DID_OFFSET) & 0xFF;
		
		if (PMS5005.GET_MOTOR_SENSOR_DATA == pms5005MessageType)
		{
			//System.err.println("-*- Motor Sensor Data Package Received -*-");
			motorSensorData.setMotorSensorData(pkg.data);
			updateHeading();
			if (null != iRobotEventHandler) 
			{
				iRobotEventHandler.sensorDataReceivedEvent(pkg.robotIP, pkg.robotPort, pms5005MessageType);
			}
		}
		else if (PMS5005.GET_CUSTOM_SENSOR_DATA == pms5005MessageType)
		{
			//System.err.println("-*- Custom Sensor Data Package Received -*-");
			customSensorData.setCustomSensorData(pkg.data);
			if (null != iRobotEventHandler) 
			{
				iRobotEventHandler.sensorDataReceivedEvent(pkg.robotIP, pkg.robotPort, pms5005MessageType);
			}
		}
		else if (PMS5005.GET_STANDARD_SENSOR_DATA == pms5005MessageType)
		{
			//System.err.println("-*- Standard Sensor Data Package Received -*-");
			standardSensorData.setStandardSensorData(pkg.data);
			if (null != iRobotEventHandler) 
			{
				iRobotEventHandler.sensorDataReceivedEvent(pkg.robotIP, pkg.robotPort, pms5005MessageType);
			}
		}
		else if (PMB5010.ADPCM_RESET == pmb5010MessageType)
		{
			////System.err.println("-*- ADPCM Reset Command Package Received -*-");
			adpcm.init();
			txBufferLength = PMB5010.ack(txBuffer, pkg.msg.get(Pkg.SEQ_OFFSET) & 0xFF);
			socket.send(txBuffer, txBufferLength);
		}
		else if (PMB5010.AUDIO_PACKAGE == pmb5010MessageType)
		{
			////System.err.println("-*- Audio Data Package Received -*-");
			if (null != iRobotEventHandler)
			{
				iRobotEventHandler.audioDataReceivedEvent(pkg.robotIP, pkg.robotPort, adpcm.decode(pkg.data, pkg.length));
			}
		}
		else if (PMB5010.VIDEO_PACKAGE == pmb5010MessageType)
		{
			////System.err.println("-*- Image Data Package Received -*-");
			if (null != iRobotEventHandler)
			{
				// Step 1: Clear buffer sizes if we're receiving first JPEG packet.
				if (PMB5010.SEQ_BEGIN == (byte) (pkg.msg.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF))
				//if (0 == this.numImagePkgs)
				{
					////System.err.println("Beginning jpeg image assembly");
					try
					{
						imageBuffer.flush();
					}
					catch (IOException e)
					{
						e.printStackTrace();
					}
					imageBuffer.reset();
				}
				
				if (0 == imageBuffer.size() || 
					PMB5010.SEQ_TERMINATE == (pkg.data.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF) || 
					previousSEQ == (pkg.data.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF) - 1)
				{
					// Step 2: Assemble complete data in buffer.
					////System.err.println("Continuing jpeg image assembly");
					imageBuffer.write(pkg.data.array(), Pkg.IMAGE_DATA_RELATIVE_OFFSET, pkg.length);
					previousSEQ = pkg.data.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF;
				}
				else if (previousSEQ != (pkg.data.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF) - 1)
				{
					//System.err.println("Warning: Image pieces out of sequence");
				}
				
				if (PMB5010.SEQ_TERMINATE == (pkg.data.get(PMB5010.VIDEO_SEQ_OFFSET) & 0xFF)) // || this.numImagePkgs - 1 <= pkg[PMB5010.VIDEO_SEQ_OFFSET])
				{
					// Step 3: Decode complete data from buffer if we have finished receiving.
					////System.err.println("Finished jpeg image assembly");
					iRobotEventHandler.imageDataReceivedEvent(pkg.robotIP, pkg.robotPort, imageBuffer);
				}
			}
			// else if null == iRobotEventHandler (no delegate), we won't do anything with the data.
		}
		else if (PMS5005.SETUP_COM == pms5005MessageType)
		{
			////System.err.println("-*- SETUP_COM Command Received -*-");
			for (int i = 0; i < pkg.length; ++i)
			{
				////System.err.printf("%2x ", (byte) (pkg.data[i] & 0xFF));
			}
			////System.err.println();
			txBufferLength = PMS5005.ack(txBuffer);
			socket.send(txBuffer, txBufferLength);
		}
	}
	
	public void socketEvent(String robotIP, int robotPort, byte[] data, int dataLength)
	{
		//System.err.println("-*- New Message -*-");
		//System.err.println("message length: " + dataLength);
		//System.err.println("pkg.offset: " + pkg.offset);
		//System.err.print("DEBUG message: ");
		for (int i = 0; i < dataLength; ++i)
		{
			//System.err.printf("%2x ", data[i] & 0xFF);
		}
		//System.err.println();
		
		int i = 0;
		while (i < dataLength)
		{
			if (i < dataLength && Pkg.STX0_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.STX0_OFFSET, data[i]);
				if (Pkg.STX0 != data[i]) 
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] STX0 isn't where it is expected to be: %2x", data[i]);
					//System.err.println();
				}
				else
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found STX0: %2x", data[i]);
					//System.err.println();
				}
				++i;
				++pkg.offset;
			}
			if (i < dataLength && Pkg.STX1_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.STX1_OFFSET, data[i]);
				if (Pkg.STX1 != data[i])
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] STX1 isn't where it is expected to be: %2x", data[i]);
					//System.err.println();
				}
				else
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found STX1: %2x", data[i]);
					//System.err.println();
				}
				++i;
				++pkg.offset;
			}
			if (i < dataLength && Pkg.RID_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.RID_OFFSET, data[i]);
				//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found RID: %2x", data[i]);
				//System.err.println();
				++i;
				++pkg.offset;
			}
			if (i < dataLength && Pkg.SEQ_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.SEQ_OFFSET, data[i]);
				//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found SEQ: %2x", data[i]);
				//System.err.println();
				++i;
				++pkg.offset;
			}
			if (i < dataLength && Pkg.DID_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.DID_OFFSET, data[i]);
				//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found DID: %2x", data[i]);
				//System.err.println();
				++i;
				++pkg.offset;
			}
			if (i < dataLength && Pkg.LENGTH_OFFSET == pkg.offset)
			{
				pkg.msg.put(Pkg.LENGTH_OFFSET, data[i]); 
				pkg.length = data[i] & 0xFF;
				//System.err.println("DEBUG: pkg[" + pkg.offset + "] Found LENGTH: " + pkg.length);
				++i;
				++pkg.offset;
				//System.err.print("DEBUG: pkg header: ");
				for (int k = 0; k < Pkg.HEADER_LENGTH; ++k)
				{
					//System.err.printf("%2x ", pkg.msg.get(k) & 0xFF);
				}
				//System.err.println();
			}
			if (i < dataLength && Pkg.DATA_OFFSET <= pkg.offset && pkg.offset < Pkg.HEADER_LENGTH + pkg.length)
			{
				int j = pkg.offset - Pkg.HEADER_LENGTH;
				//System.err.println("DEBUG: pkg.offset - Pkg.HEADER_LENGTH => j: " + j);
				//System.err.print("DEBUG: pkg[" + pkg.offset + "] Found DATA: ");
				while (i < dataLength && j < pkg.length)
				{
					pkg.msg.put(Pkg.HEADER_LENGTH + j, data[i]);
					pkg.data.put(j, data[i]);
					//System.err.printf("%2x ", pkg.data.get(j) & 0xFF);
					++i;
					++j;
					++pkg.offset;
				}
				//System.err.println();
				if (dataLength <= i)
				{
					//System.err.println("package overflowed packet in data region, last entry: ");
					//System.err.printf("pkg.data[" + (pkg.offset - 1) + "]: %2x", pkg.data.get(pkg.offset - 1) & 0xFF);
					//System.err.println();
				}
			}
			int offsetChecksum = Pkg.HEADER_LENGTH + pkg.length + Pkg.CHECKSUM_RELATIVE_OFFSET;
			if (i < dataLength && offsetChecksum == pkg.offset)
			{
				pkg.msg.put(offsetChecksum, pkg.checksum());
				if ((data[i] & 0xFF) != (pkg.checksum() & 0xFF))
				{
					//System.err.println("DEBUG: Incorrect package checksum (ignore for now)");
					//System.err.printf("DEBUG: Calculation Expected: %2x", pkg.checksum() & 0xFF);
					//System.err.println();
					//System.err.printf("DEBUG: Package Actual: %2x", data[i] & 0xFF);
					//System.err.println();
					//System.err.print("Raw data: ");
					for (int k = 0; k < pkg.length; ++k)
					{
						//System.err.printf("%2x ", pkg.data.get(k) & 0xFF);
					}
					//System.err.println();
				}
				else
				{
					////System.err.printf("DEBUG: Correct package checksum: %2x", pkg.checksum);
				}
				++i;
				++pkg.offset;
			}
			int offsetETX0 = Pkg.HEADER_LENGTH + pkg.length + Pkg.ETX0_RELATIVE_OFFSET;
			if (i < dataLength && offsetETX0 == pkg.offset)
			{
				pkg.msg.put(offsetETX0, data[i]);
				if (Pkg.ETX0 != (data[i] & 0xFF))
				{
					//System.err.printf("DEBUG: ETX0 isn't where it is expected to be (found %2x)", data[i] & 0xFF);
					//System.err.println();
				}
				else
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found ETX0: %2x", data[i] & 0xFF);
					//System.err.println();
				}
				++i;
				++pkg.offset;
			}
			int offsetETX1 = Pkg.HEADER_LENGTH + pkg.length + Pkg.ETX1_RELATIVE_OFFSET;
			if (i < dataLength && offsetETX1 == pkg.offset)
			{
				pkg.msg.put(offsetETX1, data[i]);
				if (Pkg.ETX1 != (data[i] & 0xFF))
				{
					//System.err.printf("DEBUG: ETX1 isn't where it is expected to be (found %2x)", data[i] & 0xFF);
					//System.err.println();
				}
				else
				{
					//System.err.printf("DEBUG: pkg[" + pkg.offset + "] Found ETX1: %2x, dispatching", data[i] & 0xFF);
					//System.err.println();
				}
				pkg.robotIP = robotIP;
				pkg.robotPort = robotPort;
				//System.err.println("DEBUG: len: " + dataLength + ", pkg.offset: " + pkg.offset + ", i: " + i);
				//System.err.println("DEBUG: dispatching...");
				dispatch(pkg); // or produce/enqueue
				//System.err.println("DEBUG: resetting package");
				pkg.clear();
				++i;
			}
		}
	}
	
	public void attachShutdownHook()
	{
		Runtime.getRuntime().addShutdownHook(new Thread(this));
	}
	
	private void doShutdown()
	{
		//System.err.println("Shutting down robot");
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, Motors.CONTROL_MODE_PWM); 
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, Motors.CONTROL_MODE_PWM);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, Motors.PWM_N, Motors.PWM_N, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 0);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 1);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 2);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 3);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 4);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 5);
		socket.send(txBuffer, txBufferLength);
		
		//System.err.println("Lower Head");
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, 
				(short) (Motors.SERVO_Y_INI/2 + 500), Motors.SERVO_X_INI, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
		socket.close();
	}
	
	public void shutdown()
	{
		doShutdown();
	}
	
	public void run()
	{
		doShutdown();
	}
	
	/**
	 * analog to digital conversion
	 * 
	 * @param ADValue
	 * @return distance value in meters
	 */
	public static double AD2Distance(int ADValue)
	{
		double distance = 0;
		
		if (ADValue <= 0 || 4095 < ADValue)
		{
			distance = 0;
		}
		else
		{
			distance = 21.6 / ((double) (ADValue) * 3.0 / 4028 - 0.17);
			
			if (80 < distance)
			{
				distance = 0.81;
			}
			else if (distance < 10)
			{
				distance = 0.09;
			}
			else
			{
				distance = distance / 100.0;
			}
		}
		
		return distance;
	}
	
	public void resetHead()
	{
		System.err.println("Reset Head");
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, Motors.SERVO_Y_INI, Motors.SERVO_X_INI, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		System.err.println("txBufferLength: " + txBufferLength);
		System.err.println("txBuffer: ");
		for (int i = 0; i < txBufferLength; ++i)
		{
			System.err.printf("%2x ", txBuffer[i]);
		}
		System.err.println();
		socket.send(txBuffer, txBufferLength);
		System.err.println("txBuffer sent");
		System.err.println();
	}
	
	public void lowerHead()
	{
		System.err.println("Lower Head");
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, (short) (Motors.SERVO_Y_INI/2 + 500), Motors.SERVO_X_INI, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		System.err.println("txBufferLength: " + txBufferLength);
		System.err.println("txBuffer: ");
		for (int i = 0; i < txBufferLength; ++i)
		{
			System.err.printf("%2x ", txBuffer[i]);
		}
		System.err.println();
		socket.send(txBuffer, txBufferLength);
		System.err.println("txBuffer sent");
		System.err.println();
	}
	
	public void resumeAllSensors()
	{
		//System.err.println("Activating All Sensors");
		txBufferLength = PMS5005.enableCustomSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.enableStandardSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.enableMotorSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		//System.err.println("All Sensors Activated");
	}
	
	public void suspendAllSensors()
	{
		//System.err.println("Deactivating All Sensors");
		txBufferLength = PMS5005.disableCustomSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.disableStandardSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.disableMotorSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
		//System.err.println("All Sensors Deactivated");
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
	public void motorSensorRequest(int packetNumber)
	{
		txBufferLength = PMS5005.motorSensorRequest(txBuffer, (short) packetNumber);
		socket.send(txBuffer, txBufferLength);
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
	public void standardSensorRequest(int packetNumber)
	{
		txBufferLength = PMS5005.standardSensorRequest(txBuffer, (short) packetNumber);
		socket.send(txBuffer, txBufferLength);
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
	public void customSensorRequest(int packetNumber)
	{
		txBufferLength = PMS5005.customSensorRequest(txBuffer, (short) packetNumber);
		socket.send(txBuffer, txBufferLength);
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
	public void allSensorRequest(int packetNumber)
	{
		txBufferLength = PMS5005.allSensorRequest(txBuffer, (short) packetNumber);
		socket.send(txBuffer, txBufferLength);
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
	public void enableMotorSensorSending()
	{
		txBufferLength = PMS5005.enableMotorSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
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
	public void enableStandardSensorSending()
	{
		txBufferLength = PMS5005.enableStandardSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
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
	public void enableCustomSensorSending()
	{
		txBufferLength = PMS5005.enableCustomSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
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
	public void enableAllSensorSending()
	{
		txBufferLength = PMS5005.enableAllSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Disables batch updating of motor-related sensor packets.
     *
     * @see enableMotorSensorSending
     */
	public void disableMotorSensorSending()
	{
		txBufferLength = PMS5005.disableMotorSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Disables batch updating of standard sensor packets.
     *
     * @see enableStandardSensorSending
     */
	public void disableStandardSensorSending()
	{
		txBufferLength = PMS5005.disableStandardSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Disables batch updating of custom sensor packets.
     *
     * @see enableCustomSensorSending
     */
	public void disableCustomSensorSending()
	{
		txBufferLength = PMS5005.disableCustomSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Disables batch updating of all sensor packets.
     *
     * @see enableAllSensorSending
     */
	public void disableAllSensorSending()
	{
		txBufferLength = PMS5005.disableAllSensorSending(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sets the refresh rate for batch updating by motor-related sensor packets.
     *
     * @param milliseconds The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see motorSensorRequest
     */
	public void setMotorSensorPeriod(int milliseconds)
	{
		txBufferLength = PMS5005.setMotorSensorPeriod(txBuffer, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sets the refresh rate for batch updating by standard sensor packets.
     *
     * @param milliseconds The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see standardSensorRequest
     */
	public void setStandardSensorPeriod(int milliseconds)
	{
		txBufferLength = PMS5005.setStandardSensorPeriod(txBuffer, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sets the refresh rate for batch updating by custom sensor packets.
     *
     * @param milliseconds The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see customSensorRequest
     */
	public void setCustomSensorPeriod(int milliseconds)
	{
		txBufferLength = PMS5005.setCustomSensorPeriod(txBuffer, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sets the refresh rate for batch updating of all sensor packets.
     *
     * @param milliseconds The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see allSensorRequest
     */
	public void setAllSensorPeriod(int milliseconds)
	{
		txBufferLength = PMS5005.setAllSensorPeriod(txBuffer, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Returns the current distance value between the relevant ultrasonic 
     * range sensor module (DUR5200) and the object in front of it.
     *
     * @param channel 0, 1, 2, 3, 4 or 5 for Sonar #1 through #6.
     *
     * @return 0.04 means a distance of 0.00 to 0.04 meters to object.
     * 0.04 to 2.54 means a distance of 0.04 to 2.54 meters to object.
     * 2.55 means 2.55 meters or longer distance to object.
     *
     * Please note: By default, the sensors are indexed clockwise, starting 
     * with the left-front sensor (robot first person perspective) at Sonar #1 
     * (channel 0).
     */
	public double getSonarRange(int channel)
	{
		return standardSensorData.sonarDistance[channel] / 100.0;
	}
	
    /**
     * Returns the current distance measurement value between an infrared 
     * sensor and the object in front of it.
     *
     * @param channel 0, 1, 2, 3, 4 or 5 for IR #1 through #6.
     *
     * @return distance measurement in meters. 0.09 means 0.00 to 0.09 meters to 
     * object, 0.10 through 0.80 means a distance of 0.10 to 0.80 to object, 
     * 0.81 means 0.81 meters or longer distance to object.
     */
	public double getIRRange(int channel)
	{
		double result = -1;
		if (0 == channel)
		{
			result = AD2Distance(standardSensorData.infraredRangeAD);
		}
		else if (1 <= channel)
		{
			result = AD2Distance(customSensorData.customAD[1 + channel]);
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
	public int getHumanAlarm(int channel)
	{
		return standardSensorData.humanAlarm[channel];
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
     * the direction of motion.  The relationship can be obtained emperically.
     */
	public int getHumanMotion(int channel)
	{
		return standardSensorData.motionDetect[channel];
	}
	
    /**
     * Returns the current tilt angle value in the horizontal direction from 
     * the DTA5102 Tilting and Acceleration Sensor Module.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
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
	public int getTiltingX()
	{
		return standardSensorData.tiltingAD[X];
	}
	
    /**
     * Returns the current tilt angle value in the vertical direction from 
     * the DTA5102 Tilting and Acceleration Sensor Module.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
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
	public int getTiltingY()
	{
		return standardSensorData.tiltingAD[Y];
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
	public int getOverheat(int channel)
	{
		return standardSensorData.overheatAD[channel];
	}
	
    /**
     * Returns the current temperature value from the 
     * DA5280 Ambient Temperature Sensor Module.
     *
     * @return Temperature = (ival - 1256) / 34.8, where Temperature is in 
     * degrees Celsius.
     */
	public int getAmbientTemperature()
	{
		return standardSensorData.temperatureAD;
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
	public int getIRCode(int index)
	{
		return standardSensorData.infraredCommand[index];
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
    void setInfraredControlOutput(int lowWord, int highWord)
    {
    	// TODO unimplemented
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
	public int getMainboardBatteryVoltageAD()
	{
		return standardSensorData.mainboardBatteryVoltageAD_0to9V;
	}
	
	
	public int getMotorBatteryVoltageAD()
	{
		return standardSensorData.motorBatteryVoltageAD_0to24V;
	}
	
	public int getServoBatteryVoltageAD()
	{
		return standardSensorData.servoBatteryVoltageAD_0to9V;
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
	public int getReferenceVoltage()
	{
		return standardSensorData.referenceVoltageAD_Vcc;
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
	public int getPotentiometerVoltage()
	{
		return standardSensorData.potentiometerVoltageAD_Vref;
	}
	
    /**
     * Returns the current value of the specified potentiometer position sensor
     *
     * @param channel 0, 1, 2, 3, 4, or 5 for Potentiometer sensor #1 through 6
     *
     * @return The raw value given by the analog to digital converter 
     * indicating the output voltage of the sensor.  The data range is between 
     * 0 and 4095.  The angular position can be calculated as follows, with the 
     * 180 degree position defined at the sensors' physical middle position.  
     * Single sensor or dual sensor can be used for rotation measurement.
     *
     * Please note:
     * 1) Single sensor mode is mainly used for the control of a robot joint 
     *    with a limited rotation range.  The effective mechanical rotation 
     *    range is 14 degrees to 346 degrees, corresponding to the effective 
     *    electrical rotation range 0 degrees to 332 degrees.
     *      Angle position (degrees) = (ival - 2048)/4095*333 + 180
     * 2) Dual-sensor mode is mainly used for continuous rotating joint control 
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
     * @see setDCMotorSensorUsage
     */
	public int getPotentiometer(int channel)
	{
		return motorSensorData.potentiometerAD[channel];
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
	public int getMotorCurrent(int channel)
	{
		return motorSensorData.motorCurrentAD[channel];
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
	public int getEncoderDirection(int channel)
	{
		return motorSensorData.encoderDirection[channel];
	}
	
    /**
     * Returns the current pulse counter to indicate the position of rotation.
     *
     * @param channel 0 for left encoder, 1 for right encoder (robot first 
     * person perspective).
     *
     * @return Pulse counter, an integral value to rotation with range of 
     * 0 to 32767 in cycles.
     */
	public int getEncoderPulse(int channel)
	{
		return motorSensorData.encoderPulse[channel];
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
	public int getEncoderSpeed(int channel)
	{
		return motorSensorData.encoderSpeed[channel];
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
     * @see getBatteryAD
     */
	public int getCustomAD(int channel)
	{
		return customSensorData.customAD[channel];
	}
	
    /**
     * Returns a value with the lower 8 bits corresponding to the 8 channel 
     * custom digital inputs.
     *
     * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8.
     */
	public int getCustomDIn(int channel)
	{
		return 0;
	}
	
    /**
     * Sets the 8-channel custom digital outputs.
     *
     * @param ival -- only the lower 8 bits are valid and can change the 
     * corresponding outputs of the 8 channels.  The MSB of the lower byte 
     * represents channel #8 and LSB of the lower byte represents channel #1.
     */
	public void setCustomDOut(int ival)
	{
		txBufferLength = PMS5005.setCustomDOut(txBuffer, (byte) ival);
		socket.send(txBuffer, txBufferLength);
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
	public void setMotorPolarity(int channel, int polarity)
	{
		txBufferLength = PMS5005.setMotorPolarity(txBuffer, (byte) channel, (byte) polarity);
		socket.send(txBuffer, txBufferLength);
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
	public void enableDCMotor(int channel)
	{
		txBufferLength = PMS5005.enableDCMotor(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
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
	public void disableDCMotor(int channel)
	{
		txBufferLength = PMS5005.disableDCMotor(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Resumes the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     */
	public void resumeDCMotor(int channel)
	{
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Resumes the DC motor control channels.
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public void resumeAllDCMotors()
	{
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 0);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 1);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 2);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 3);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 4);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, (byte) 5);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Resumes the DC motor control channels.
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public void resumeBothDCMotors()
	{
		//System.err.println("Resume Both DC Motors");
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, L);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.resumeDCMotor(txBuffer, R);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Suspends the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public void suspendDCMotor(int channel)
	{
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Suspends the DC motor control channels.
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public void suspendAllDCMotors()
	{
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 0);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 1);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 2);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 3);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 4);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, (byte) 5);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Suspends the DC motor control channels.
     *
     * All motor control channels are initially suspended at boot-up.
     */
	public void suspendBothDCMotors()
	{
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, L);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.suspendDCMotor(txBuffer, R);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sets up the PID control parameters of the specified DC motor channel 
     * for position control.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     * @param kp proportional gain (default is 50)
     * @param kd derivative gain (default is 5)
     * @param ki_x100 the desired integral gain * 100.  when ki_100 = 100, 
     * the actual integral control term is ki = 1.  ki_x100 has a range of 
     * 0 to 25599, where 0 means no integral control (default).
     *
     * @see setDCMotorControlMode
     */
	public void setDCMotorPositionControlPID(int channel, int kp, int kd, int ki_x100)
	{
		txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, (byte) channel, (short) kp, (short) kd, (short) ki_x100);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * This filtering feature is still under development. All data will be 
     * treated as raw data.
     */
	public void setDCMotorSensorFilter(int channel, int filterMethod)
	{
		txBufferLength = PMS5005.setDCMotorSensorFilter(txBuffer, (byte) channel, (short) filterMethod);
		socket.send(txBuffer, txBufferLength);
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
     * @see getPotentiometerReading
     */
	public void setDCMotorSensorUsage(int channel, int sensorType)
	{
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, (byte) channel, (byte) sensorType);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void setBothDCMotorSensorUsages(int sensorTypeL, int sensorTypeR)
	{
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, (byte) 0, (byte) sensorTypeL);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, (byte) 0, (byte) sensorTypeR);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void setBothDCMotorSensorUsages(int sensorType)
	{
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, L, (byte) sensorType);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorSensorUsage(txBuffer, R, (byte) sensorType);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void setBothDCMotorControlModes(int controlModeL, int controlModeR)
	{
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, (byte) controlModeL);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, (byte) controlModeR);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void setBothDCMotorControlModes(int controlMode)
	{
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, (byte) controlMode);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, (byte) controlMode);
		socket.send(txBuffer, txBufferLength);
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
	public void setDCMotorControlMode(int channel, int controlMode)
	{
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, (byte) channel, (byte) controlMode);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the position control command to the specified motion control 
     * channel on the Sensing and Motion Controller (PMS5005).  
     * The command includes the target position and the target time 
     * period to execute the command.  The current trajectory planning method 
     * with time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param position Target position value
     * @param milliseconds Executing time in milliseconds
     */
	public void setDCMotorPosition(int channel, int position, int milliseconds)
	{
		txBufferLength = PMS5005.setDCMotorPosition(txBuffer, (byte) channel, (short) position, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
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
     * @see getPotentiometerReading
     */
	public void setDCMotorPosition(int channel, int position)
	{
		txBufferLength = PMS5005.setDCMotorPosition(txBuffer, (byte) channel, (short) position);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to the specified motion control channel on 
     * the Sensing and Motion Controller (PMS5005).  The command includes the 
     * target pulse width value and the time period to execute the command. 
     * The current trajectory planning method for time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param pulseWidth Target pulse width value
     * @param milliseconds Executing time in milliseconds
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
     */
	public void setDCMotorPulse(int channel, int pulseWidth, int milliseconds)
	{
		txBufferLength = PMS5005.setDCMotorPulse(txBuffer, (byte) channel, (short) pulseWidth, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
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
     */
	public void setDCMotorPulse(int channel, int pulseWidth)
	{
		txBufferLength = PMS5005.setDCMotorPulse(txBuffer, (byte) channel, (short) pulseWidth);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the position control command to all 6 DC motor control channels on 
     * the sensing and motion controller (PMS5005) at the same time.  The 
     * command includes the target positions and the time period to execute the 
     * command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param position0 Target position for channel #1
     * @param position1 Target position for channel #2
     * @param position2 Target position for channel #3
     * @param position3 Target position for channel #4
     * @param position4 Target position for channel #5
     * @param position5 Target position for channel #6
     * @param milliseconds Execution time in milliseconds
     * 
     * 1) All DC Motors will be enabled automatically by the system when this 
     *    command is received.
     * 2) Target position value is the A/D sampling data range 0 to 4095 when 
     *    using single potentiometer, 0-4428 when using dual potentiometers.
     * 3) Please refer to the description of getPotentiometerReading for data 
     *    conversion between angular values and the A/D sampling data values.
     * 4) When using the encoder as sensor input, the target position value is 
     *    the pulse count in the range of 0-32767.
     * 5) When omitting motor channels from control, the command value should 
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     */
	public void setAllDCMotorPositions(int position0, int position1, int position2, int position3, int position4, int position5, int milliseconds)
	{
		txBufferLength = PMS5005.setAllDCMotorPositions(txBuffer, 
				(short) position0, (short) position1, (short) position2, 
				(short) position3, (short) position4, (short) position5,
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the position control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target positions without a specified time period 
     * for execution.  The motion controller will drive the motor to reach the 
     * target position with maximum effort.
     * 
     * @param position0 Target position for channel #1
     * @param position1 Target position for channel #2
     * @param position2 Target position for channel #3
     * @param position3 Target position for channel #4
     * @param position4 Target position for channel #5
     * @param position5 Target position for channel #6
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
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     */
	public void setAllDCMotorPositions(int position0, int position1, int position2, int position3, int position4, int position5)
	{
		
		txBufferLength = PMS5005.setAllDCMotorPositions(txBuffer, (short) position0, (short) position1, (short) position2, (short) position3, (short) position4, (short) position5);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the velocity control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target velocities and the time period to execute 
     * the command.  The trajectory planning method for time control is linear.
     * 
     * @param velocity0 Target velocity for channel #1
     * @param velocity1 Target velocity for channel #2
     * @param velocity2 Target velocity for channel #3
     * @param velocity3 Target velocity for channel #4
     * @param velocity4 Target velocity for channel #5
     * @param velocity5 Target velocity for channel #6
     * @param milliseconds Execution time in milliseconds
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
     *    (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setAllDCMotorVelocities(int velocity0, int velocity1, int velocity2, int velocity3, int velocity4, int velocity5, int milliseconds)
	{
		txBufferLength = PMS5005.setAllDCMotorVelocities(txBuffer, 
				(short) velocity0, (short) velocity1, (short) velocity2, 
				(short) velocity3, (short) velocity4, (short) velocity5,
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the velocity control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target velocities without specifying an execution 
     * time period.  The motion controller will drive the motor to achieve 
     * the target velocity with maximum effort.
     * 
     * @param velocity0 Target velocity for channel #1
     * @param velocity1 Target velocity for channel #2
     * @param velocity2 Target velocity for channel #3
     * @param velocity3 Target velocity for channel #4
     * @param velocity4 Target velocity for channel #5
     * @param velocity5 Target velocity for channel #6
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
     *    (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setAllDCMotorVelocities(int velocity0, int velocity1, int velocity2, int velocity3, int velocity4, int velocity5)
	{
		txBufferLength = PMS5005.setAllDCMotorVelocities(txBuffer, 
				(short) velocity0, (short) velocity1, (short) velocity2, 
				(short) velocity3, (short) velocity4, (short) velocity5);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to all 6 DC motor control channels on the 
     * Sensing and Motion Controller (PMS5005) at the same time.  The command 
     * includes the target PWM values and the time period for execution.  The 
     * current trajectory planning method for time control is linear.
     * 
     * @param pulseWidth0 Target PWM value for channel #1
     * @param pulseWidth1 Target PWM value for channel #2
     * @param pulseWidth2 Target PWM value for channel #3
     * @param pulseWidth3 Target PWM value for channel #4
     * @param pulseWidth4 Target PWM value for channel #5
     * @param pulseWidth5 Target PWM value for channel #6
     * @param milliseconds Execution time in milliseconds
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
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
	public void setAllDCMotorPulses(int pulseWidth0, int pulseWidth1, int pulseWidth2, 
			int pulseWidth3, int pulseWidth4, int pulseWidth5, int milliseconds)
	{
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, 
				(short) pulseWidth0, (short) pulseWidth1, (short) pulseWidth2, 
				(short) pulseWidth3, (short) pulseWidth4, (short) pulseWidth5,
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to all 6 DC motor control channels on the 
     * Sensing and Motion Controller (PMS5005) at the same time.  The command 
     * includes the target PWM values without a specified time period for 
     * execution.  The motion controller will adjust the pulse width right away.
     * 
     * @param pulseWidth0 Target PWM value for channel #1
     * @param pulseWidth1 Target PWM value for channel #2
     * @param pulseWidth2 Target PWM value for channel #3
     * @param pulseWidth3 Target PWM value for channel #4
     * @param pulseWidth4 Target PWM value for channel #5
     * @param pulseWidth5 Target PWM value for channel #6
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
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
	public void setAllDCMotorPulses(int pulseWidth0, int pulseWidth1, int pulseWidth2, 
			int pulseWidth3, int pulseWidth4, int pulseWidth5)
	{
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, (short) pulseWidth0, (short) pulseWidth1, (short) pulseWidth2, (short) pulseWidth3, (short) pulseWidth4, (short) pulseWidth5);
		socket.send(txBuffer, txBufferLength);
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
	public void enableServo(int channel)
	{
		txBufferLength = PMS5005.enableServo(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
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
	public void disableServo(int channel)
	{
		txBufferLength = PMS5005.disableServo(txBuffer, (byte) channel);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command and the time period to execute the 
     * command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param pulseWidth Target Pulse Width (in milliseconds) * 2250
     * @param milliseconds Executing time in milliseconds
     * 
     * Usually, a standard remote control servo motor expects to get the 
     * specified pulse width every 20 seconds in order to hold the 
     * corresponding angle position.  The pulse width value in milliseconds 
     * for 0 degrees, 90 degrees, and 180 degrees are servo manufacturer and 
     * model dependent.  They are around 1ms, 1.5ms, and 2.0ms respectively for 
     * most common servos.  Experiments are required to obtain the exact value 
     * for a specific servo motor.
     * 
     */
	public void setServoPulse(int channel, int pulseWidth, int milliseconds)
	{
		txBufferLength = PMS5005.setServoPulse(txBuffer, (byte) channel, (short) pulseWidth, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command without a specific time period for 
     * execution.  The motion controller will send the desired pulse width to 
     * the servo motor right away.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param pulseWidth Target pulse width (ms) * 2250
     * 
     */
	public void setServoPulse(int channel, int pulseWidth)
	{
		txBufferLength = PMS5005.setServoPulse(txBuffer, (byte) channel, (short) pulseWidth);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to all 6 servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at the 
     * same time.
     *
     * The command includes the target position commands and the 
     * time period to execute the command.  The current trajectory planning 
     * method for time control is linear.
     *
     * @param pulseWidth0 Target pulse width for channel #1 (Left Motor on X80Pro)
     * @param pulseWidth1 Target pulse width for channel #2 (-Right Motor on X80Pro)
     * @param pulseWidth2 Target pulse width for channel #3 (NO_CONTROL on X80Pro)
     * @param pulseWidth3 Target pulse width for channel #4 (NO_CONTROL on X80Pro)
     * @param pulseWidth4 Target pulse width for channel #5 (NO_CONTROL on X80Pro)
     * @param pulseWidth5 Target pulse width for channel #6 (NO_CONTROL on X80Pro)
     * @param milliseconds Executing time in milliseconds
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setAllServoPulses(int pulseWidth0, int pulseWidth1, int pulseWidth2, 
			int pulseWidth3, int pulseWidth4, int pulseWidth5, int milliseconds)
	{
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, (short) pulseWidth0, (short) pulseWidth1, (short) pulseWidth2, (short) pulseWidth3, (short) pulseWidth4, (short) pulseWidth5, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends PWM control commands to all 6 servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at once. 
     *
     * The command includes the target position commands without specifying a 
     * time period in which to execute the command.  The motion controller 
     * sends the desired pulse width to the servo motor right away.
     *
     * @param pulseWidth0 Target position for channel #1 (Left Motor on X80Pro)
     * @param pulseWidth1 Target position for channel #2 (-Right Motor on X80Pro)
     * @param pulseWidth2 Target position for channel #3 (NO_CONTROL on X80Pro)
     * @param pulseWidth3 Target position for channel #4 (NO_CONTROL on X80Pro)
     * @param pulseWidth4 Target position for channel #5 (NO_CONTROL on X80Pro)
     * @param pulseWidth5 Target position for channel #6 (NO_CONTROL on X80Pro)
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setAllServoPulses(int pulseWidth0, int pulseWidth1, int pulseWidth2, int pulseWidth3, int pulseWidth4, int pulseWidth5)
	{
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, (short) pulseWidth0, (short) pulseWidth1, (short) pulseWidth2, (short) pulseWidth3, (short) pulseWidth4, (short) pulseWidth5);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Displays the image data in the file bmpFileName (BMP format) on the 
     * graphic LCD connected to the Sensing and Motion Controller (PMS5005).
     * 
     * @param bmpFileName Full path of the BMP file for displaying
     * 
     * The graphic LCD display is monochrome with dimensions 128 by 64 pixels.  
     * The bitmap (bmp) image must be 128x64 pixels in mono.
     */
	public void setLCDDisplayPMS(BufferedImage bufferedImage) throws IOException
	{
		ImageIO.setUseCache(false);
		ByteArrayOutputStream imageBaos = new ByteArrayOutputStream();
		ImageIO.write(bufferedImage, "bmp", imageBaos);
		imageBaos.flush();
		byte[] imageBytes = imageBaos.toByteArray();
		byte[] imageBytesFor8Rows = new byte[64];
		imageBaos.close();
		/* send image buffer slices */
		for (byte frameSlice = 0; frameSlice < (imageBytes.length >> 6); ++frameSlice)
		{
			for (int j = 0; j < imageBytesFor8Rows.length; ++j)
			{
				imageBytesFor8Rows[j] = (byte) (imageBytes[(frameSlice << 6) + j] & 0xFF);
			}
			txBufferLength = PMS5005.setLCDDisplayPMS(txBuffer, imageBytesFor8Rows, frameSlice);
			socket.send(txBuffer, txBufferLength);
		}
	}
	
    /**
     * Sets up the PID control parameters of the specified DC motor channel 
     * for velocity control.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     * @param kp proportional gain (default is 50)
     * @param kd derivative gain (default is 5)
     * @param ki_x100 the desired integral gain * 100.  when ki_100 = 100, 
     * the actual integral control term is ki = 1.  ki_x100 has a range of 
     * 0 to 25599, where 0 means no integral control (default).
     *
     * @see setDCMotorControlMode
     */
	public void setDCMotorVelocityControlPID(int channel, int kp, int kd, int ki)
	{
		txBufferLength = PMS5005.setDCMotorVelocityControlPID(txBuffer, (byte) channel, (short) kp, (short) kd, (short) ki);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the position control command to both (the first two) DC motor control 
     * channels on the sensing and motion controller (PMS5005) at the same time.  
     * The command includes the target positions and the time period to execute 
     * the command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param positionLeft Target position for channel #1
     * @param positionRight Target position for channel #2
     * @param milliseconds Execution time in milliseconds
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
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     */
	public void setBothDCMotorPositions(int positionLeft, int positionRight, int milliseconds) 
	{
		txBufferLength = PMS5005.setAllDCMotorPositions(txBuffer, (short) positionLeft, (short) -positionRight, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the position control command to both (the first two) DC motor control 
     * channels on the sensing and motion controller (PMS5005) at the same time.  
     * The command includes the target positions and the time period to execute 
     * the command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param positionLeft Target position for channel #1
     * @param positionRight Target position for channel #2
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
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     */
	public void setBothDCMotorPositions(int positionLeft, int positionRight) 
	{
		txBufferLength = PMS5005.setAllDCMotorPositions(txBuffer, (short) positionLeft, (short) -positionRight, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the velocity control command to all both (the first two) DC motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at the same 
     * time.  The command includes the target velocities and the time period to 
     * execute the command.  The trajectory planning method for time control is 
     * linear.
     * 
     * @param leftVelocity Target velocity for channel #1
     * @param rightVelocity Target velocity for channel #2
     * @param milliseconds Execution time in milliseconds
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
     *    (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setBothDCMotorVelocities(int leftVelocity, int rightVelocity, int milliseconds) 
	{
		txBufferLength = PMS5005.setAllDCMotorVelocities(txBuffer, (short) leftVelocity, (short) -rightVelocity, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the velocity control command to both (the first two) DC motor control 
     * channels on the Sensing and Motion Controller (PMS5005) at the same time.  
     * The command includes the target velocities without specifying an execution 
     * time period.  The motion controller will drive the motor to achieve 
     * the target velocity with maximum effort.
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
     *    (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setBothDCMotorVelocities(int leftVelocity, int rightVelocity) 
	{
		txBufferLength = PMS5005.setAllDCMotorVelocities(txBuffer, (short) leftVelocity, (short) -rightVelocity, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to both (the first two) DC motor control 
     * channels on the Sensing and Motion Controller (PMS5005) at the same time.  
     * The command includes the target PWM values and the time period for 
     * execution.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param leftPulseWidth Target PWM value for channel #1
     * @param rightPulseWidth Target PWM value for channel #2
     * @param milliseconds Execution time in milliseconds
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
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
	public void setBothDCMotorPulses(int leftPulseWidth, int rightPulseWidth, int milliseconds) 
	{
		txBufferLength = PMS5005.setAllDCMotorVelocities(txBuffer, (short) leftPulseWidth, (short) -rightPulseWidth, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to both (the first two) DC motor control 
     * channels on the Sensing and Motion Controller (PMS5005) at the same time.  
     * The command includes the target PWM values without a specified time period 
     * for execution.  The motion controller will adjust the pulse width right 
     * away.
     * 
     * @param leftPulseWidth Target PWM value for channel #1
     * @param rightPulseWidth Target PWM value for channel #2
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
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
	public void setBothDCMotorPulses(int leftPulseWidth, int rightPulseWidth) 
	{
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, (short) leftPulseWidth, (short) -rightPulseWidth, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends the PWM control command to both (the first two) servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at the 
     * same time.
     *
     * The command includes the target position commands and the 
     * time period to execute the command.  The current trajectory planning 
     * method for time control is linear.
     *
     * @param verticalPulseWidth Target pulse width for channel #1 (Left Motor on X80Pro)
     * @param horizontalPulseWidth Target pulse width for channel #2 (-Right Motor on X80Pro)
     * @param milliseconds Executing time in milliseconds
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setBothServoPulses(int verticalPulseWidth, int horizontalPulseWidth, int milliseconds) 
	{
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, (short) verticalPulseWidth, (short) horizontalPulseWidth, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
				(short) milliseconds);
		socket.send(txBuffer, txBufferLength);
	}
	
    /**
     * Sends PWM control commands to both (the first two) servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at once. 
     *
     * The command includes the target position commands without specifying a 
     * time period in which to execute the command.  The motion controller 
     * sends the desired pulse width to the servo motor right away.
     *
     * @param verticalPulseWidth Target position for channel #1 (Left Motor on X80Pro)
     * @param horizontalPulseWidth Target position for channel #2 (-Right Motor on X80Pro)
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     */
	public void setBothServoPulses(int verticalPulseWidth, int horizontalPulseWidth) 
	{
		txBufferLength = PMS5005.setAllServoPulses(txBuffer, (short) verticalPulseWidth, (short) horizontalPulseWidth, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void startAudioRecording(int voiceSegmentLength)
	{
		txBufferLength = PMB5010.startAudioRecording(txBuffer, (byte) voiceSegmentLength);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void stopAudioRecording()
	{
		txBufferLength = PMB5010.stopAudioRecording(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void startAudioPlayback(int sampleLength)
	{
		for (byte seq = 0; seq < sampleLength / 0xff; ++seq)
		{
			txBufferLength = PMB5010.startAudioPlayback(txBuffer, (short) sampleLength, seq);
			socket.send(txBuffer, txBufferLength);
		}
		
		txBufferLength = PMB5010.startAudioPlayback(txBuffer, (short) sampleLength, (byte) (0xff & 0xff));
		socket.send(txBuffer, txBufferLength);
	}
	
	public void stopAudioPlayback()
	{
		txBufferLength = PMB5010.stopAudioPlayback(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void continueAudioPlayback(byte[] audioSegmentEncodedInPCM)
	{
		txBufferLength = PMB5010.continueAudioPlayback(txBuffer, audioSegmentEncodedInPCM);
		socket.send(txBuffer, txBufferLength);
	}
	
	public void takePhoto()
	{
		txBufferLength = PMB5010.takePhoto(txBuffer);
		socket.send(txBuffer, txBufferLength);
	}
	
	/**
	 * This method is taken directly from the DrRobot Java Demo Project
	 * 
	 * Method sets Position Control Mode, and turns robot through theta radians in time seconds
	 * @param theta angle to turn through in radians
	 * @param time time to turn in seconds
	 */
	public void turnStep(double theta, int milliseconds)
	{
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, X80Pro.Motors.CONTROL_MODE_POSITION);
	    socket.send(txBuffer, txBufferLength);
	    txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, X80Pro.Motors.CONTROL_MODE_POSITION);
	    socket.send(txBuffer, txBufferLength);
	    
	    // s = r*theta*(circumferenceInMetersToCircumferenceInEncoderUnitsConversion)
	    int s = (int) ((X80Pro.WHEEL_CIRCUMFERENCE_ENCODER_COUNT*WHEEL_DISPLACEMENT*theta)/(4*Math.PI*WHEEL_RADIUS));
	    
	    int leftPosition = absEncoderPulseValue(motorSensorData.encoderPulse[L] - s);
	    int rightPosition = absEncoderPulseValue(motorSensorData.encoderPulse[R] - s);
	    
	    txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, L, (short) 1000, (short) 5, (short) 10000);
	    socket.send(txBuffer, txBufferLength);
	    
	    txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, R, (short) 1000, (short) 5, (short) 10000);
	    socket.send(txBuffer, txBufferLength);
	    
	    PMS5005.setAllDCMotorPositions(txBuffer, (short) leftPosition, (short) rightPosition, 
	    		Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
	    		(short) milliseconds);
	    socket.send(txBuffer, txBufferLength);
	}
	
	public void runStepPWM(double runDis, int milliseconds)
	{
        int d = (int)((runDis/(2*Math.PI*WHEEL_RADIUS))*WHEEL_CIRCUMFERENCE_ENCODER_COUNT);
        
        int LeftTarget = absEncoderPulseValue(motorSensorData.encoderPulse[L] + d);
        int RightTarget = absEncoderPulseValue(motorSensorData.encoderPulse[R] - d);
        
        txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, X80Pro.Motors.CONTROL_MODE_PWM);
        socket.send(txBuffer, txBufferLength);
        txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, X80Pro.Motors.CONTROL_MODE_PWM);
        socket.send(txBuffer, txBufferLength);
        
        //socket.send(PMS5005.setDCMotorPositionControlPID((byte) L, (short) 1000, (short) 30, (short) 2000));
        //socket.send(PMS5005.setDCMotorPositionControlPID((byte) R, (short) 1000, (short) 30, (short) 2000));
        
        txBufferLength = PMS5005.setAllDCMotorPulses(
        		txBuffer, 
        		(short) (Motors.PWM_N + Motors.PWM_O + 8*Motors.DUTY_CYCLE_UNIT), 
        		(short) -(Motors.PWM_N + Motors.PWM_O + 8*Motors.DUTY_CYCLE_UNIT), 
        		Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
        		(short) milliseconds);
        socket.send(txBuffer, txBufferLength);
        while (motorSensorData.encoderPulse[L] < LeftTarget - Sensors.ACCEPTABLE_ENCODER_DEVIATION || 
        	   motorSensorData.encoderPulse[R] < RightTarget - Sensors.ACCEPTABLE_ENCODER_DEVIATION)
        {
        	// Busy wait
        }
        
	}
	
	/**
	 * This method is taken directly from the DrRobot Java Demo Project
	 * 
	 * @param runDis
	 */
    public void runStep(double runDis, int milliseconds) 
    {
        //the robot will go forward the rundistance
        int diffEncoder = (int)((runDis/(2*Math.PI*WHEEL_RADIUS))*WHEEL_CIRCUMFERENCE_ENCODER_COUNT);
        
        int LeftTarget = absEncoderPulseValue(motorSensorData.encoderPulse[L] + diffEncoder);
        int RightTarget = absEncoderPulseValue(motorSensorData.encoderPulse[R] - diffEncoder);
        
        txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, X80Pro.Motors.CONTROL_MODE_POSITION);
        socket.send(txBuffer, txBufferLength);
        txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, (byte) X80Pro.Motors.CONTROL_MODE_POSITION);
        socket.send(txBuffer, txBufferLength);
        
        txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, L, (short) 1000, (short) 30, (short) 2000);
        socket.send(txBuffer, txBufferLength);
        txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, R, (short) 1000, (short) 30, (short) 2000);
        socket.send(txBuffer, txBufferLength);
        
        PMS5005.setAllDCMotorPositions(txBuffer, (short) LeftTarget, (short) RightTarget, 
        		Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, 
        		(short) milliseconds);
        socket.send(txBuffer, txBufferLength);
    }
	
	public int turnThetaRadianStep(double theta, int milliseconds)
	{
		//@ post: Robot has turned an angle theta in radians
		
		double r = WHEEL_DISPLACEMENT/2.0;
		double metersToEncoderUnitsConversion = WHEEL_CIRCUMFERENCE_ENCODER_COUNT/(2*Math.PI*WHEEL_RADIUS);
	    int s = (int) (r*theta*metersToEncoderUnitsConversion);
	    
	    int leftPulseWidth = absEncoderPulseValue(getEncoderPulse(L) - s);
		int rightPulseWidth = absEncoderPulseValue(getEncoderPulse(R) - s);
		
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, L, X80Pro.Motors.CONTROL_MODE_PWM);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorControlMode(txBuffer, R, X80Pro.Motors.CONTROL_MODE_PWM);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, L, (short) 1000, (short) 5, (short) 10000);
		socket.send(txBuffer, txBufferLength);
		txBufferLength = PMS5005.setDCMotorPositionControlPID(txBuffer, R, (short) 1000, (short) 5, (short) 10000);
		socket.send(txBuffer, txBufferLength);
		
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, (short) leftPulseWidth, (short) -rightPulseWidth, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, (short) milliseconds);
		socket.send(txBuffer, txBufferLength);
		return leftPulseWidth;
	}
	
	/**
	 * 
	 * @param percentageLeft represents the left pulse width percentage, from 0 to 100
	 * @param percentageRight represents the right pulse width percentage, from 0 to 100
	 */
	public void setBothDCMotorPulsePercentages(int percentageLeft, int percentageRight)
	{
		double powerLeft = percentageLeft/100.0; // 0.0 <= x <= 1.0
		double powerRight = percentageRight/100.0; // 0.0 <= x <= 1.0
		
		if (0 < powerLeft)
		{
			// 16384 + 8000 + (scaled) power[L]: increase left motor velocity.
			powerLeft = Motors.PWM_N + Motors.PWM_O + powerLeft*(Motors.DUTY_CYCLE_UNIT/3.0);
		}
		else if (powerLeft < 0)
		{
			// 16384 - 8000 + (scaled) power[L]: reduce left motor velocity.
			powerLeft = Motors.PWM_N - Motors.PWM_O - powerLeft*(Motors.DUTY_CYCLE_UNIT/3.0); 
		}
		else
		{
			// neutral PWM setting, 0% duty cycle, left motor is idle
			powerLeft = Motors.PWM_N;
		}
		
		//if (powerLeft > L_MAX_PWM)
		//{
		// L_MAX_PWM = 32767, +100% duty cycle
		//	powerLeft = L_MAX_PWM;
		//}
		//else if (powerLeft < N_PWM)
		//{
		// stand still, 0% duty cycle
		//	powerLeft = N_PWM;
		//}
		
		if (0 < powerRight)
		{
			powerRight = Motors.PWM_N - Motors.PWM_O - powerRight*(Motors.DUTY_CYCLE_UNIT/3.0); // reverse: 16384 - 8000 - rightPulseWidth
		}
		else if (powerRight < 0)
		{
			powerRight = Motors.PWM_N + Motors.PWM_O + powerRight*(Motors.DUTY_CYCLE_UNIT/3.0); // reverse: 16384 + 8000 + rightPulseWidth
		}
		else
		{
			powerRight = Motors.PWM_N; // neutral PWM setting, 0% duty cycle
		}
		//if (powerRight < R_MAX_PWM) powerRight = R_MAX_PWM; // yes, < .. negative threshold @ -100% duty cycle
		//else if (powerRight > N_PWM) powerRight = N_PWM; // stand still, 0% duty cycle
		//robot.dcMotorPwmNonTimeCtrlAll((int)powerLeft, (int)powerRight, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
		
		txBufferLength = PMS5005.setAllDCMotorPulses(txBuffer, (short) powerLeft, (short) powerRight, 
				Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL, Motors.NO_CTRL);
		socket.send(txBuffer, txBufferLength);
	}
	
	private int absEncoderPulseValue(int encoderPulse)
	{
		if (encoderPulse < 0) encoderPulse += 32767;
		else if (32767 < encoderPulse) encoderPulse -= 32768;
		return encoderPulse;
	}
	
	/**
	 * Resets heading to zero by storing current left and right encoder pulse values as initial values 
	 */
	public void resetHeading()
	{	
		encoderPulseInitial[L] = absEncoderPulseValue(motorSensorData.encoderPulse[L]);
		encoderPulseInitial[R] = absEncoderPulseValue(motorSensorData.encoderPulse[R]);
	}
	
	private void updateHeading()
	{
		// s = r*theta
		double r = WHEEL_DISPLACEMENT/2.0;
		
		int encoderPulseLeft = absEncoderPulseValue(motorSensorData.encoderPulse[L]);
		int encoderPulseRight = absEncoderPulseValue(motorSensorData.encoderPulse[R]);
		
		int sLeft = encoderPulseLeft - encoderPulseInitial[L];
		int sRight = encoderPulseRight - encoderPulseInitial[R];
		
		double thetaLeft = (sLeft/r)*((2*Math.PI*WHEEL_RADIUS)/WHEEL_CIRCUMFERENCE_ENCODER_COUNT);
		double thetaRight = (sRight/r)*((2*Math.PI*WHEEL_RADIUS)/WHEEL_CIRCUMFERENCE_ENCODER_COUNT);
		
		heading = (thetaLeft + thetaRight) / 2.0;
	}
	
	/**
	 * 
	 * @return heading value in radians
	 */
	public double getHeading()
	{
		return heading;
	}
	
	/**
	 * Called by worker thread 
	 * @param robotIP
	 * @param robotPort
	 * @param maneuverID
	 */
	@Override
	public void progressCompletedEvent(String robotIP, int robotPort)
	{
		busyFlag = false;
	}
	
	@Override
	public boolean isBusy()
	{
		return busyFlag;
	}
	
	/**
	 * Reports robot busy state progress, returns a value 0 to 100 representing a percentage.
	 */
	@Override
	public int reportProgress()
	{
		return busyProgress;
	}
	
	@Override
	public void startProgress()
	{
		busyFlag = true;
	}
}
