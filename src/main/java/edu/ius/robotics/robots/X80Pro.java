package edu.ius.robotics.robots;

import java.io.IOException;
import java.lang.Runtime;
import java.util.Arrays;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.boards.PMB5010;
import edu.ius.robotics.robots.codecs.X80ProADPCM;
import edu.ius.robotics.robots.codecs.nanojpeg.NanoJPEGCodec;
import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotAudio;
import edu.ius.robotics.robots.interfaces.IRobotVideo;
import edu.ius.robotics.robots.interfaces.IX80Pro;

import edu.ius.robotics.robots.sockets.UDPSocket;

public class X80Pro implements IX80Pro, IRobot, Runnable
{
	/** minimum time step in milliseconds */
	public final int MIN_TIME_MILLI = 50;
	
	public final int nStdSensors = 6;
	public final int nMotors = 2;
	public final int nIrSensors = 8;
	public final int nCustomSensors = 9;
	public final int nUltrasonicSensors = 6;
	public final int nHumanSensors = 2;
	
	public static final int DEFAULT_ROBOT_PORT = 10001;
	public static final int NO_CTRL = -32768;
	
	public static final int DEFAULT_MOTOR_SENSOR_TIME_PERIOD = 50;
	public static final int DEFAULT_STANDARD_SENSOR_TIME_PERIOD = 50;
	public static final int DEFAULT_CUSTOM_SENSOR_TIME_PERIOD = 50;
	
	public static final int ACCEPTABLE_ENCODER_DEVIATION = 16;
	public static final int NO_CONTROL = -32768;
	
	public static final int CONTROL_MODE_PWM = 0;
	public static final int CONTROL_MODE_POSITION = 1;
	public static final int CONTROL_MODE_VELOCITY = 2;
	
	public static final int SENSOR_USAGE_SINGLE_POTENTIOMETER = 0x00;
	public static final int SENSOR_USAGE_DUAL_POTENTIOMETER = 0x01;
	public static final int SENSOR_USAGE_ENCODER = 0x02;
	
	public static final int INITIAL_TARGET_FINAL = 3;
	public static final int FINAL = 2;
	public static final int TARGET = 1;
	public static final int INITIAL = 0;
	public static final int TURN_QUAD_T = 4;

	public static final int MODE_SINGLEPOT = 0;
	public static final int MODE_DUALPOT = 1;
	public static final int MODE_QUADRATURE = 2;

	public static final int cFULL_COUNT = 32767;
	public static final int cWHOLE_RANGE = 1200;

	public static final int DIRECTION = -1;
	public static final int SERVO0_INI = 3650;
	public static final int SERVO1_INI = 3300;
	public static final int MAX_VELOCITY = 1200;
	public static final int VELOCITY = 1000;
	
	public static final int MICRO_DELAY = 50;
	public static final int MINI_DELAY = 100;
	public static final int DELAY = 500;
	public static final int BIG_DELAY = 1000;
	public static final int UBER_DELAY = 5000;
	
	public static final int MAX_PWM_L = 32767; // 32767;
	public static final int MAX_PWM_R = -0; // 0;
	
	public static final int PWM_N = 16383;
	public static final int PWM_O = 8000;
	public static final int DUTY_CYCLE_UNIT = 8383;
	
	public static final short WHEEL_ENCODER_2PI = 800;
	
	public static final double MAX_IR_DIS = 0.81;
	public static final double MIN_IR_DIS = 0.09;
	public static final short MAX_IR_DIS_IN_CM = 81;
	public static final short MIN_IR_DIS_IN_CM = 9;
	
	public static final int L = 0;
	public static final int R = 1;
	
	public static final int NUM_IR_SENSORS = 7;
	public static final int NUM_IR_SENSORS_FRONT = 4;
	public static final int MAX_SPEED = 32767;
	public static final int RANGE = 30;
	
	public static class Sensors 
	{
		public static Double[] thetaIR = new Double[4]; // Sensor displacements in radians
		
		static 
		{
			thetaIR[0] = 0.785;
			thetaIR[1] = 0.35;
			thetaIR[2] = -0.35;
			thetaIR[3] = -0.785;
		}
	}
	
	// private ByteArrayInputStream byteArrIn;
	// private DataInputStream dataIn;
	// private DataOutputStream dataOut;
	// private InputStream inputStream;
	// private BufferedReader reader;
	// private String command;
	/** wheel distance */
	public static final double WHEEL_CIRCUMFERENCE = 0.265; // meters
	public static final double WHEEL_DISPLACEMENT = 0.2897; // meters (default: 0.305)
	
	/** wheel radius */
	public static final double WHEEL_RADIUS = 0.0825; // meters
	
	/** encoder one circle count */
	public static final int CIRCLE_ENCODER_COUNT = 1200;
	
	/** meh */
	public int[] encoderPos;
	public double[] irDis;
	// motor sensor
	public int[] encoderSpeed;
	public double[] motorCurrent;
	// custom sensor data
	public int[] customAd;
	public int customIo;
	// standard sensor data
	public double[] usDis;
	public int[]    humanAlarm;
	public int[]    humanMotion;
	public int      irRange;
	public double   boardVol;
	public double   dcMotorVol;
	
//	private int motorSensorTimePeriod;
//	private int standardSensorTimePeriod;
//	private int customSensorTimePeriod;
	
	volatile private byte[] motorSensorData;
	volatile private byte[] standardSensorData;
	volatile private byte[] customSensorData;
	//private int[] powerSensorData;
	
	volatile private byte[][] oldMotorSensorData;
	volatile private byte[][] oldStandardSensorData;
	volatile private byte[][] oldCustomSensorData;
	
	//private boolean[] lockIRRange;
	byte previousSEQ;
	
	byte[] audioBuffer;
	byte[] jpegBuffer;
	
	int audioBufferSize;
	int jpegBufferSize;
	
	private UDPSocket socket;
	private X80ProADPCM pcm;
	private NanoJPEGCodec jpeg;
	private IRobotAudio iRobotAudio;
	private IRobotVideo iRobotVideo;

	private void preInit()
	{
		//lockIRRange = new boolean[NUM_IR_SENSORS];
		this.iRobotAudio = null;
		this.iRobotVideo = null;
		
		this.jpegBuffer = new byte[16384]; // 16K
		this.audioBuffer = new byte[16384]; // 16K
		this.previousSEQ = 0;
		this.jpegBufferSize = 0;
		this.audioBufferSize = 0;
		
		this.motorSensorData = new byte[PMS5005.HEADER_LENGTH + PMS5005.MOTOR_SENSOR_DATA_LENGTH];
		this.customSensorData = new byte[PMS5005.HEADER_LENGTH + PMS5005.CUSTOM_SENSOR_DATA_LENGTH];
		this.standardSensorData = new byte[PMS5005.HEADER_LENGTH + PMS5005.STANDARD_SENSOR_DATA_LENGTH];
		
		// The following few lines are a bad hack to compare the last two sensor readings from each different type.
		// This is to find out what the rate of change is, so that we can possibly correct weird sensor behavior.
		this.oldMotorSensorData = new byte[2][];
		this.oldCustomSensorData = new byte[2][];
		this.oldStandardSensorData = new byte[2][];
		
		this.oldMotorSensorData[0] = this.motorSensorData;
		this.oldCustomSensorData[0] = this.customSensorData;
		this.oldStandardSensorData[0] = this.standardSensorData;
	}
	
	private void postInit()
	{
		// Enable collection of all sensor data by default.
		this.socket.send(PMS5005.enableMotorSensorSending());
		this.socket.send(PMS5005.enableCustomSensorSending());
		this.socket.send(PMS5005.enableStandardSensorSending());
		
		// This shutdown hook indicates other threads should terminate as well (i.e. sensor data collection thread).
		this.attachShutdownHook();
	}
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param robotIp
	 */
	public X80Pro(String ipAddress) throws IOException
	{
		preInit();
		this.socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		postInit();
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
		this.socket = new UDPSocket(this, ipAddress, port);
		postInit();
	}
	
	public X80Pro(String ipAddress, IRobotAudio iRobotAudio, IRobotVideo iRobotVideo) throws IOException
	{
		preInit();
		this.iRobotAudio = iRobotAudio;
		this.iRobotVideo = iRobotVideo;
		this.socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		postInit();
	}
	
	public X80Pro(String ipAddress, int port, IRobotAudio iRobotAudio, IRobotVideo iRobotVideo) throws IOException
	{
		preInit();
		this.iRobotAudio = iRobotAudio;
		this.iRobotVideo = iRobotVideo;
		this.socket = new UDPSocket(this, ipAddress, port);
		postInit();
	}
	
	/**
	 * sends a command to the robot
	 * 
	 * @param command
	 */
	public void sendCommand(byte[] command)
	{
		socket.send(command);
	}
	
	/**
	 * clearSensorData
	 */
	public void clearSensorData()
	{
		int c, z;
		
		for (c = 0, z = this.encoderPos.length; c < z; ++c)
		{
			this.encoderPos[c] = 0;
		}
		
		for (c = 0, z = this.irDis.length; c < z; ++c)
		{
			this.irDis[c] = (double) (c + 1);
		}
		
		// motor sensor
		for (c = 0, z = this.encoderSpeed.length; c < z; ++c)
		{
			this.encoderSpeed[c] = 0;
		}
		
		for (c = 0, z = this.motorCurrent.length; c < z; ++c)
		{
			this.motorCurrent[c] = 0.0;
		}
		
		// custom sensor data
		for (c = 0, z = this.customAd.length; c < z; ++c)
		{
			this.customAd[c] = 0;
		}
		
		this.customIo = 0;
		
		// standard sensor data
		for (c = 0, z = this.usDis.length; c < z; ++c)
		{
			this.usDis[c] = 0;
		}
		
		for (c = 0, z = this.humanAlarm.length; c < z; ++c)
		{
			this.humanAlarm[c] = 0;
		}
		
		for (c = 0, z = this.humanMotion.length; c < z; ++c)
		{
			this.humanMotion[c] = 0;
		}
		
		this.irRange = 0;
		this.boardVol = 0;
		this.dcMotorVol = 0;
	}
	
	public void dumpSensorData()
	{
		int c, z;
		
		System.err.print("encoderPos: ");
		for (c = 0, z = this.encoderPos.length; c < z; ++c)
		{
			System.err.print(this.encoderPos[c] + " ");
		}
		System.err.println();
		
		System.err.print("irDis: ");
		for (c = 0, z = this.irDis.length; c < z; ++c)
		{
			System.err.print(this.irDis[c] + " ");
		}
		System.err.println();
		
		// motor sensor
		System.err.print("encoderSpeed: ");
		for (c = 0, z = this.encoderSpeed.length; c < z; ++c)
		{
			System.err.print(this.encoderSpeed[c] + " ");
		}
		System.err.println();
		
		System.err.println("motorCurrent: ");
		for (c = 0, z = this.motorCurrent.length; c < z; ++c)
		{
			System.err.print(this.motorCurrent[c] + " ");
		}
		System.err.println();
		
		// custom sensor data
		for (c = 0, z = this.customAd.length; c < z; ++c)
		{
			this.customAd[c] = 0;
		}
		
		this.customIo = 0;
		
		// standard sensor data
		for (c = 0, z = this.usDis.length; c < z; ++c)
		{
			this.usDis[c] = 0;
		}
		
		for (c = 0, z = this.humanAlarm.length; c < z; ++c)
		{
			this.humanAlarm[c] = 0;
		}
		
		for (c = 0, z = this.humanMotion.length; c < z; ++c)
		{
			this.humanMotion[c] = 0;
		}
		
		this.irRange = 0;
		this.boardVol = 0;
		this.dcMotorVol = 0;
	}
	
	public void sensorEvent(String robotIP, int robotPort, byte[] sensorData)
	{
		int z = sensorData.length;
		
		if (sensorData[0] == PMS5005.STX0 && sensorData[1] == PMS5005.STX1 && 
			sensorData[z-2] == PMS5005.ETX0 && sensorData[z-1] == PMS5005.ETX1)
		{
			// here is a whole package (all sensor data)
			// for first motor sensor data, please refer to the protocol
			// documentation
			if (PMS5005.GET_MOTOR_SENSOR_DATA == sensorData[PMS5005.DID_OFFSET])
			{
//				System.err.println("-*- RECEIVED MOTOR SENSOR DATA -*-");
//				System.err.println("length: " + z);
				//int c = 0;
				//for (int i = 0, y = sensorData.length; i < y; ++i)
				//{
				//	this.motorSensorData[i] = sensorData[i];
				//	System.err.print(sensorData[i]);
				//}
				//System.err.println();
				this.oldMotorSensorData[1] = this.oldMotorSensorData[0];
				this.oldMotorSensorData[0] = this.motorSensorData;
				this.motorSensorData = sensorData;
//				System.err.print("content: ");
//				for (int x : sensorData)
//				{
//					System.err.print(x + " ");
//				}
//				System.err.println();
			}
			else if (PMS5005.GET_CUSTOM_SENSOR_DATA == sensorData[PMS5005.DID_OFFSET])
			{
//				System.err.println("-*- RECEIVED CUSTOM SENSOR DATA -*-");
//				System.err.println("length: " + z);
//				this.customSensorData = Arrays.copyOfRange(sensorData, PMS5005.HEADER_LENGTH, sensorData.length);
//				int c = 0;
				//for (int i = 0, y = sensorData.length; i < y; ++i)
				//{
				//	this.customSensorData[i] = sensorData[i];
					//System.err.print("DEBUG: " + sensorData[i]);
				//}
				//System.err.println("DEBUG: ");
				this.oldCustomSensorData[1] = this.oldCustomSensorData[0];
				this.oldCustomSensorData[0] = this.customSensorData;
				this.customSensorData = sensorData;
				/* Patch sensor behavior */
//				for (int channel = 1; channel < 7; ++channel)
//				{
//					short oldestIRRange = ((short) (((oldCustomSensorData[1][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (oldCustomSensorData[1][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
//					short oldIRRange = ((short) (((oldCustomSensorData[0][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (oldCustomSensorData[0][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
//					short newIRRange = ((short) (((customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
//					if (AD2Dis(oldestIRRange) < AD2Dis(oldIRRange) && 0.09 == AD2Dis(newIRRange))
//					{
//						customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] = (byte) (oldCustomSensorData[0][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff);
//						customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] = (byte) (oldCustomSensorData[0][2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] & 0xff);
//					}
//				}

//				System.err.print("content: ");
//				for (int x : sensorData)
//				{
//					System.err.print(x + " ");
//				}
//				System.err.println();
			}
			else if (PMS5005.GET_STANDARD_SENSOR_DATA == sensorData[PMS5005.DID_OFFSET])
			{
				//System.err.println("-*- RECEIVED STANDARD SENSOR DATA -*-");
				//System.err.println("length: " + z);
				//this.standardSensorData = Arrays.copyOfRange(sensorData, PMS5005.HEADER_LENGTH, sensorData.length);
//				int c = 0;
				//for (int i = 0, y = sensorData.length; i < y; ++i)
				//{
				//	this.standardSensorData[i] = sensorData[i];
					//System.err.print("DEBUG: " + sensorData[i]);
				//}
				//System.err.println("DEBUG: ");
				this.oldStandardSensorData[1] = this.oldStandardSensorData[0];
				this.oldStandardSensorData[0] = this.standardSensorData;
				this.standardSensorData = sensorData;
				/* Patch sensor behavior */
				short oldestIRRange = ((short) (((oldStandardSensorData[1][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (oldStandardSensorData[1][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
				short oldIRRange = ((short) (((oldStandardSensorData[0][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (oldStandardSensorData[0][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
				short newIRRange = ((short) (((standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
				if (AD2Dis(oldestIRRange) < AD2Dis(oldIRRange) || 0.81 == AD2Dis(oldIRRange) && 0.09 == AD2Dis(newIRRange))
				{
					standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] = (byte) (oldStandardSensorData[0][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff);
					standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] = (byte) (oldStandardSensorData[0][PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff);
				}

				//				System.err.print("content: ");
//				for (int x : sensorData)
//				{
//					System.err.print(x + " ");
//				}
//				System.err.println();
			}
			else if (PMB5010.ADPCM_RESET == sensorData[PMB5010.DID_OFFSET])
			{
				pcm.init();
				// respond with ACK? ... socket.send(PMB5010.ack());
			}
			else if (PMB5010.AUDIO_PACKET == sensorData[PMB5010.DID_OFFSET])
			{
				// we just received audio stream data, what do we do with it?
				if (null != iRobotAudio)
				{
					iRobotAudio.audioEvent(robotIP, robotPort, pcm.decode(Arrays.copyOfRange(sensorData, PMB5010.PAYLOAD_OFFSET, sensorData.length - PMB5010.FOOTER_LENGTH), sensorData[PMB5010.LENGTH_OFFSET]));					
				}
			}
			else if (PMB5010.VIDEO_PACKET == sensorData[PMB5010.DID_OFFSET])
			{
				if (null != iRobotVideo)
				{
					// Step 1: Clear buffer sizes if we're receiving first JPEG packet.
					if (PMB5010.SEQ_BEGIN == sensorData[PMB5010.SEQ_OFFSET])
					{
						jpegBufferSize = 0;
					}
					
					// Step 2: Assemble complete data in buffer.
					if (0 == jpegBufferSize || previousSEQ + 1 == sensorData[PMB5010.SEQ_OFFSET])
					{
						for (int i = PMB5010.PAYLOAD_OFFSET; i < sensorData.length - PMB5010.FOOTER_LENGTH; ++i)
						{
							jpegBuffer[jpegBufferSize + (i - PMB5010.PAYLOAD_OFFSET)] = sensorData[i];
						}
						
						jpegBufferSize += sensorData.length - PMB5010.METADATA_SIZE;
					}
					
					// Step 3: Decode complete data from buffer if we have finished receiving.
					if (PMB5010.SEQ_TERMINATE == sensorData[PMB5010.SEQ_OFFSET])
					{
						//iRobotVideo.videoEvent(robotIP, robotPort, jpeg.decode(jpegBuffer, jpegBufferSize));
					}
				} // else if null == iRobotVideo (no delegate), we won't do anything with the data.
			}
		}
	}
	
	public void attachShutdownHook()
	{
		Runtime.getRuntime().addShutdownHook(new Thread(this));
	}
	
	public void shutdown()
	{
		System.err.println("Shutting down robot");
		
		socket.send(PMS5005.suspendDCMotor((byte) 0));
		socket.send(PMS5005.suspendDCMotor((byte) 1));
		socket.send(PMS5005.suspendDCMotor((byte) 2));
		socket.send(PMS5005.suspendDCMotor((byte) 3));
		socket.send(PMS5005.suspendDCMotor((byte) 4));
		socket.send(PMS5005.suspendDCMotor((byte) 5));
		
		socket.close();
	}
	
	public void run()
	{
		this.suspendAllDCMotors();
		this.lowerHead();
		socket.close();
	}
	
	/**
	 * analog to digital conversion
	 * 
	 * @param ADValue
	 * @return distance value in meters
	 */
	public static double AD2Dis(int ADValue)
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
		socket.send(PMS5005.setAllServoPulses((short) SERVO0_INI, (short) SERVO1_INI, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}
	
	public void lowerHead()
	{
		System.err.println("Lower Head");
		socket.send(PMS5005.setAllServoPulses((short) (SERVO0_INI/2), (short) (SERVO1_INI), (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}
	
	public void resumeAllSensors()
	{
		System.err.println("Activating All Sensors");
		
		socket.send(PMS5005.enableCustomSensorSending());
		socket.send(PMS5005.enableStandardSensorSending());
		socket.send(PMS5005.enableMotorSensorSending());
		
		System.err.println("All Sensors Activated");
	}
	
	public void motorSensorRequest(int packetNumber)
	{
		socket.send(PMS5005.motorSensorRequest((short) packetNumber));
	}
	
	public void standardSensorRequest(int packetNumber)
	{
		socket.send(PMS5005.standardSensorRequest((short) packetNumber));
	}
	
	public void customSensorRequest(int packetNumber)
	{
		socket.send(PMS5005.customSensorRequest((short) packetNumber));
	}
	
	public void allSensorRequest(int packetNumber)
	{
		socket.send(PMS5005.allSensorRequest((short) packetNumber));
	}
	
	public void enableMotorSensorSending()
	{
		socket.send(PMS5005.enableMotorSensorSending());
	}
	
	public void enableStandardSensorSending()
	{
		socket.send(PMS5005.enableStandardSensorSending());
	}
	
	public void enableCustomSensorSending()
	{
		socket.send(PMS5005.enableCustomSensorSending());
	}
	
	public void enableAllSensorSending()
	{
		socket.send(PMS5005.enableAllSensorSending());
	}
	
	public void disableMotorSensorSending()
	{
		socket.send(PMS5005.disableMotorSensorSending());
	}
	
	public void disableStandardSensorSending()
	{
		socket.send(PMS5005.disableStandardSensorSending());
	}
	
	public void disableCustomSensorSending()
	{
		socket.send(PMS5005.disableCustomSensorSending());
	}
	
	public void disableAllSensorSending()
	{
		socket.send(PMS5005.disableAllSensorSending());
	}
	
	public void setMotorSensorPeriod(int timePeriod)
	{
		socket.send(PMS5005.setMotorSensorPeriod((short) timePeriod));
	}
	
	public void setStandardSensorPeriod(int timePeriod)
	{
		socket.send(PMS5005.setStandardSensorPeriod((short) timePeriod));
	}
	
	public void setCustomSensorPeriod(int timePeriod)
	{
		socket.send(PMS5005.setCustomSensorPeriod((short) timePeriod));
	}
	
	public void setAllSensorPeriod(int timePeriod)
	{
		socket.send(PMS5005.setAllSensorPeriod((short) timePeriod));
	}
	
//	public int getSonar(int channel)
//	{
//		return (byte) (standardSensorData[channel + PMS5005.ULTRASONIC_OFFSET] & 0xff);
//	}
	
	public double getSonarRange(int channel)
	{
		return (standardSensorData[channel + PMS5005.ULTRASONIC_OFFSET_WITH_HEADER] & 0xff) / 100.0;
	}
	
	public double getIRRange(int channel)
	{
		double result = -1;
		
		if (0 == channel)
		{
			//System.err.println("standardSensorData[24]: " + ((byte) (standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER+1] & 0x0f)));
			//System.err.println("standardSensorData[25]: " + ((byte) (standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));			
			result = AD2Dis((short) (((standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
		}
		else if (1 <= channel && channel < 7)
		{
			result = AD2Dis((short) (((customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8) | (customSensorData[2*(channel-1) + PMS5005.CUSTOM_IR_RANGE_OFFSET_WITH_HEADER] & 0xff)));
		}
		
		return result;
	}
	
	public int getHumanAlarm(int channel)
	{
		int offset = 2*channel + PMS5005.HUMAN_ALARM_OFFSET_WITH_HEADER;
		return (short) (((standardSensorData[offset + 1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getHumanMotion(int channel)
	{
		int offset = 2*channel + PMS5005.HUMAN_MOTION_OFFSET_WITH_HEADER;
		return (short) (((standardSensorData[offset+1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getTiltingX(int channel)
	{
		return (short) (((standardSensorData[PMS5005.TILTING_X_OFFSET_WITH_HEADER+1] & 0xff) << 8) | (standardSensorData[PMS5005.TILTING_X_OFFSET_WITH_HEADER] & 0xff));
	}
	
	public int getTiltingY(int channel)
	{
		return (short) (((standardSensorData[PMS5005.TILTING_Y_OFFSET_WITH_HEADER+1] & 0xff) << 8) | (standardSensorData[PMS5005.TILTING_Y_OFFSET_WITH_HEADER] & 0xff));
	}
	
	public int getOverheat(int channel)
	{
		int offset = 2*channel + PMS5005.OVERHEAT_SENSOR_OFFSET_WITH_HEADER;
		return (short) (((standardSensorData[offset+1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getAmbientTemperature()
	{
		return (short) (((standardSensorData[PMS5005.TEMPERATURE_AD_OFFSET_WITH_HEADER+1] & 0xff) << 8) | (standardSensorData[PMS5005.TEMPERATURE_AD_OFFSET_WITH_HEADER] & 0xff));
	}
	
	public int getIRCode(int index)
	{
		return (short) standardSensorData[PMS5005.INFRARED_COMMAND_OFFSET_WITH_HEADER + index];
	}
	
	public void setInfraredControlOutput(int lowWord, int highWord)
	{
		//PMS5005.setInfraredControlOutput((short) lowWord, (short) highWord);
	}
	
	public int getBatteryAD(int channel)
	{
		return (short) (((standardSensorData[2 * channel + PMS5005.BATTERY_SENSOR_OFFSET_WITH_HEADER+1] & 0xff) << 8 | standardSensorData[2*channel + PMS5005.BATTERY_SENSOR_OFFSET_WITH_HEADER] & 0xff));

	}
	
	public int getReferenceVoltage()
	{
		return (short) (((standardSensorData[PMS5005.REFERENCE_VOLTAGE_OFFSET_WITH_HEADER + 1] & 0xff) << 8 | standardSensorData[PMS5005.REFERENCE_VOLTAGE_OFFSET_WITH_HEADER] & 0xff));
	}
	
	public int getPotentiometerVoltage()
	{
		return (short) (((standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET_WITH_HEADER + 1] & 0xff) << 8 | standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET_WITH_HEADER] & 0xff));
	}
	
	public int getPotentiometerReading(int channel)
	{
		return (short) (((motorSensorData[2 * channel + PMS5005.POTENTIOMETER_SENSOR_OFFSET_WITH_HEADER + 1]) << 8 | motorSensorData[2 * channel + PMS5005.POTENTIOMETER_SENSOR_OFFSET_WITH_HEADER]));

	}
	
	public int getMotorCurrent(int channel)
	{
		return (short) ((((motorSensorData[2 * channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET_WITH_HEADER + 1]) << 8 | motorSensorData[2 * channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET_WITH_HEADER])) / 728.0);
	}
	
	public int getEncoderDirection(int channel)
	{
		int offset = channel + PMS5005.ENCODER_DIRECTION_OFFSET_WITH_HEADER;
		short result = -1;
		
		switch (channel)
		{
		case 0:
			result = (short) (motorSensorData[offset] & 0x01);
			break;
		case 1:
			result = (short) (motorSensorData[offset] & 0x03);
			break;
		}
		
		return result;
	}
	
	public int getEncoderPulse(int channel)
	{
		int offset = 4*channel + PMS5005.ENCODER_PULSE_OFFSET_WITH_HEADER;
		return (short) (((motorSensorData[offset + 1] & 0xff) << 8) | (motorSensorData[offset] & 0xff));
	}
	
	public int getEncoderSpeed(int channel)
	{
		int offset = 4*channel + PMS5005.MOTOR_SPEED_OFFSET_WITH_HEADER;
		return (short) (((motorSensorData[offset + 1] & 0xff) << 8) | (motorSensorData[offset] & 0xff));
	}
	
	public int getCustomAD(int channel)
	{
		int offset = 2*channel + PMS5005.CUSTOM_AD_OFFSET_WITH_HEADER;
		return (short) (((customSensorData[offset + 1] & 0xff) << 8) | (customSensorData[offset] & 0xff));
	}
	
	public int getCustomDIn(int channel)
	{
		return 0;
	}
	
	public void setCustomDOut(int ival)
	{
		socket.send(PMS5005.setCustomDOut((byte) ival));
	}
	
	public void setMotorPolarity(int channel, int polarity)
	{
		socket.send(PMS5005.setMotorPolarity((byte) channel, (byte) polarity));
	}
	
	@SuppressWarnings("deprecation")
	public void enableDCMotor(int channel)
	{
		socket.send(PMS5005.enableDCMotor((byte) channel));
	}
	
	@SuppressWarnings("deprecation")
	public void disableDCMotor(int channel)
	{
		socket.send(PMS5005.disableDCMotor((byte) channel));
	}
	
	public void resumeDCMotor(int channel)
	{
		socket.send(PMS5005.resumeDCMotor((byte) channel));
	}

	public void resumeAllDCMotors()
	{
		socket.send(PMS5005.resumeDCMotor((byte) 0));
		socket.send(PMS5005.resumeDCMotor((byte) 1));
		socket.send(PMS5005.resumeDCMotor((byte) 2));
		socket.send(PMS5005.resumeDCMotor((byte) 3));
		socket.send(PMS5005.resumeDCMotor((byte) 4));
		socket.send(PMS5005.resumeDCMotor((byte) 5));
	}

	public void resumeBothDCMotors()
	{
		System.err.println("Resume Both DC Motors");
		socket.send(PMS5005.resumeDCMotor((byte) 0));
		socket.send(PMS5005.resumeDCMotor((byte) 1));
	}
	
	public void suspendDCMotor(int channel)
	{
		socket.send(PMS5005.suspendDCMotor((byte) channel));
	}
	
	public void suspendAllDCMotors()
	{
		socket.send(PMS5005.suspendDCMotor((byte) 0));
		socket.send(PMS5005.suspendDCMotor((byte) 1));
		socket.send(PMS5005.suspendDCMotor((byte) 2));
		socket.send(PMS5005.suspendDCMotor((byte) 3));
		socket.send(PMS5005.suspendDCMotor((byte) 4));
		socket.send(PMS5005.suspendDCMotor((byte) 5));
	}
	
	public void suspendBothDCMotors()
	{
		System.err.println("Suspend Both DC Motors");
		socket.send(PMS5005.suspendDCMotor((byte) 0));
		socket.send(PMS5005.suspendDCMotor((byte) 1));
	}
	
	public void setDCMotorPositionControlPID(int channel, int kp, int kd, int ki_x100)
	{
		socket.send(PMS5005.setDCMotorPositionControlPID((byte) channel, (short) kp, (short) kd, (short) ki_x100));
	}
	
	public void setDCMotorSensorFilter(int channel, int filterMethod)
	{
		socket.send(PMS5005.setDCMotorSensorFilter((byte) channel, (short) filterMethod));
	}
	
	public void setDCMotorSensorUsage(int channel, int sensorType)
	{
		socket.send(PMS5005.setDCMotorSensorUsage((byte) channel, (byte) sensorType));
	}
	
	public void setBothDCMotorSensorUsages(int sensorTypeL, int sensorTypeR)
	{
		socket.send(PMS5005.setDCMotorSensorUsage((byte) 0, (byte) sensorTypeL));
		socket.send(PMS5005.setDCMotorSensorUsage((byte) 0, (byte) sensorTypeR));
	}
	
	public void setBothDCMotorSensorUsages(int sensorType)
	{
		socket.send(PMS5005.setDCMotorSensorUsage((byte) 0, (byte) sensorType));
		socket.send(PMS5005.setDCMotorSensorUsage((byte) 0, (byte) sensorType));
	}
	
	public void setBothDCMotorControlModes(int controlMode)
	{
		socket.send(PMS5005.setDCMotorControlMode((byte) 0, (byte) controlMode));
		socket.send(PMS5005.setDCMotorControlMode((byte) 1, (byte) controlMode));
	}
	
	public void setDCMotorControlMode(int channel, int controlMode)
	{
		socket.send(PMS5005.setDCMotorControlMode((byte) channel, (byte) controlMode));
	}
	
	public void setBothDCMotorControlModes(int controlModeL, int controlModeR)
	{
		socket.send(PMS5005.setDCMotorControlMode((byte) 0, (byte) controlModeL));
		socket.send(PMS5005.setDCMotorControlMode((byte) 1, (byte) controlModeR));
	}
	
	public void setDCMotorPosition(int channel, int pos, int timePeriod)
	{
		socket.send(PMS5005.setDCMotorPosition((byte) channel, (short) pos, (short) timePeriod));
	}
	
	public void setDCMotorPosition(int channel, int pos)
	{
		socket.send(PMS5005.setDCMotorPosition((byte) channel, (short) pos));
	}
	
	public void setDCMotorPulse(int channel, int pulseWidth, int timePeriod)
	{
		socket.send(PMS5005.setDCMotorPulse((byte) channel, (short) pulseWidth, (short) timePeriod));
	}
	
	public void setDCMotorPulse(int channel, int pulseWidth)
	{
		socket.send(PMS5005.setDCMotorPulse((byte) channel, (short) pulseWidth));
	}
	
	public void setAllDCMotorPositions(int pos0, int pos1, int pos2, int pos3, int pos4, int pos5, int timePeriod)
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5,
				(short) timePeriod));
	}
	
	public void setAllDCMotorPositions(int pos0, int pos1, int pos2, int pos3, int pos4, int pos5)
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5));
	}
	
	public void setAllDCMotorVelocities(int v0, int v1, int v2, int v3, int v4, int v5, int timePeriod)
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) v1, (short) v2, (short) v3, (short) v4, (short) v5,
				(short) timePeriod));
	}
	
	public void setAllDCMotorVelocities(int v0, int v1, int v2, int v3, int v4, int v5)
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) v1, (short) v2, (short) v3, (short) v4, (short) v5));
	}
	
	public void setAllDCMotorPulses(int p0, int p1, int p2, int p3, int p4, int p5, int timePeriod)
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) p1, (short) p2, (short) p3, (short) p4, (short) p5,
				(short) timePeriod));
	}
	
	public void setAllDCMotorPulses(int p0, int p1, int p2, int p3, int p4, int p5)
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) p1, (short) p2, (short) p3, (short) p4, (short) p5));
	}
	
	public void enableServo(int channel)
	{
		socket.send(PMS5005.enableServo((byte) channel));
	}
	
	public void disableServo(int channel)
	{
		socket.send(PMS5005.disableServo((byte) channel));
	}
	
	public void setServoPulse(int channel, int pos, int timePeriod)
	{
		socket.send(PMS5005.setServoPulse((byte) channel, (short) pos, (short) timePeriod));
	}
	
	public void setServoPulse(int channel, int pulseWidth)
	{
		socket.send(PMS5005.setServoPulse((byte) channel, (short) pulseWidth));
	}
	
	public void setAllServoPulses(int p0, int p1, int p2, int p3, int p4, int p5, int timePeriod)
	{
		socket.send(PMS5005.setAllServoPulses((short) p0, (short) p1, (short) p2, (short) p3, (short) p4, (short) p5, (short) timePeriod));
	}
	
	public void setAllServoPulses(int p0, int p1, int p2, int p3, int p4, int p5)
	{
		socket.send(PMS5005.setAllServoPulses((short) p0, (short) p1, (short) p2, (short) p3, (short) p4, (short) p5));
	}
	
	public void setLCDDisplayPMS(String bmpFileName)
	{
		/* TODO slice buffer */
		
		/* TODO send buffer */
		
		/*
		for (int i = 0; i < 16; ++i)
		{
			socket.send(PMS5005.setLCDDisplayPMS(frameNumber, frameContent));
		}
		*/
	}
	
	public void setDCMotorVelocityControlPID(byte channel, int kp, int kd, int ki)
	{
		socket.send(PMS5005.setDCMotorVelocityControlPID(channel, kp, kd, ki));
	}

	public void setBothDCMotorPositions(int pos0, int pos1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) -pos1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorPositions(int pos0, int pos1) 
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) -pos1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothDCMotorVelocities(int v0, int v1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) -v1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorVelocities(int v0, int v1) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) -v1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothDCMotorPulses(int p0, int p1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) p0, (short) -p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorPulses(int p0, int p1) 
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) -p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothServoPulses(int p0, int p1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) -p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}
	
	public void setBothServoPulses(int p0, int p1) 
	{
		socket.send(PMS5005.setAllServoPulses((short) p0, (short) -p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}
	
	public void startAudioRecording(byte voiceSegmentLength)
	{
		socket.send(PMB5010.startAudioRecording(voiceSegmentLength));
	}
	
	public void stopAudioRecording(byte voiceSegmentLength)
	{
		socket.send(PMB5010.stopAudioRecording(voiceSegmentLength));
	}
	
	public void startAudioPlayback(short sampleLength)
	{
		for (byte seq = 0; seq < sampleLength / 0xff; ++seq)
		{
			socket.send(PMB5010.startAudioPlayback(sampleLength, seq));
		}
		
		socket.send(PMB5010.startAudioPlayback(sampleLength, (byte) (0xff & 0xff)));
	}
	
	public void stopAudioPlayback()
	{
		socket.send(PMB5010.stopAudioPlayback());
	}
	
	public void continueAudioPlayback(short[] audioSample)
	{
		socket.send(PMB5010.continueAudioPlayback(pcm.encode(audioSample, (short) audioSample.length)));
	}
	
	/**
	 * Method sets Position Control Mode, and turns robot through theta radians in time seconds
	 * @param theta angle to turn through in radians
	 * @param time time to turn in seconds
	 */
	public void turn(double theta, int time)
	{
		socket.send(PMS5005.setDCMotorSensorUsage((byte) L, (byte) X80Pro.SENSOR_USAGE_ENCODER));
		socket.send(PMS5005.setDCMotorSensorUsage((byte) R, (byte) X80Pro.SENSOR_USAGE_ENCODER));
		
	    socket.send(PMS5005.setDCMotorControlMode((byte) L, (byte) X80Pro.CONTROL_MODE_POSITION));
	    socket.send(PMS5005.setDCMotorControlMode((byte) R, (byte) X80Pro.CONTROL_MODE_POSITION));
	    
	    int diffEncoder = (int)((WHEEL_ENCODER_2PI*WHEEL_DISPLACEMENT*theta)/(4*Math.PI*WHEEL_RADIUS));
	    
	    int leftPosition = getEncoderPulse(0) - diffEncoder;
	    if (leftPosition < 0) 
	    {
	    	leftPosition = 32767 + leftPosition;
	    }
	    else if (leftPosition > 32767)
	    {
	    	leftPosition = leftPosition - 32767;
	    }
	    
	    int rightPosition = getEncoderPulse(1) - diffEncoder;
	    if (rightPosition < 0)
	    {
	    	rightPosition = 32767 + rightPosition;
	    }
	    else if (rightPosition > 32767)
	    {
	    	rightPosition = rightPosition - 32767;
	    }
	    
	    setDCMotorPositionControlPID(L, 1000, 5, 10000);
	    setDCMotorPositionControlPID(R, 1000, 5, 10000);
	    
	    setAllDCMotorPositions((short)leftPosition, (short)rightPosition, 
	    		NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL, 1000*time);
	    
	    
	}
	
	public int turnThetaRadians(double theta)
	{
		//@ post: Robot has turned an angle theta in radians
		
		socket.send(PMS5005.setDCMotorControlMode((byte) L, (byte) CONTROL_MODE_PWM));
		socket.send(PMS5005.setDCMotorControlMode((byte) R, (byte) CONTROL_MODE_PWM));
		
	    int diffEncoder = (int)((WHEEL_ENCODER_2PI*WHEEL_DISPLACEMENT*theta)/(4*Math.PI*WHEEL_RADIUS));
	    
	    // TODO: cross calling
	    int leftPulseWidth = getEncoderPulse(L) - diffEncoder;
		if (leftPulseWidth < 0)
		{
			leftPulseWidth = 32767 + leftPulseWidth;
		}
		else if (32767 < leftPulseWidth)
		{
			leftPulseWidth = leftPulseWidth - 32767;
		}
		
		// TODO: cross calling
		int rightPulseWidth = getEncoderPulse(R) - diffEncoder;
		if (rightPulseWidth < 0) 
		{
			rightPulseWidth = 32767 + rightPulseWidth;
		}
		else if (32767 < rightPulseWidth)
		{
			rightPulseWidth = rightPulseWidth - 32767;
		}
		
		socket.send(PMS5005.setDCMotorPositionControlPID((byte) L, (short) 1000, (short) 5, (short) 10000));
		socket.send(PMS5005.setDCMotorPositionControlPID((byte) R, (short) 1000, (short) 5, (short) 10000));
		
		socket.send(PMS5005.setAllDCMotorPulses((short) leftPulseWidth, (short) -rightPulseWidth, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
		
		return leftPulseWidth;
	}
	
	public void setBothDCMotorPulsePercentages(int leftPulseWidth, int rightPulseWidth)
	{
		double[] power = new double[2];
		
		power[L] = leftPulseWidth/100.0; // 0.0 <= x <= 1.0
		power[R] = rightPulseWidth/100.0; // 0.0 <= x <= 1.0
		
		System.err.println("translatedLeftPulseWidth: " + power[L]);
		System.err.println("translatedRightPulseWidth: " + power[R]);
		
		// if > 100% power output requested, cap the motor power.
//		if (1 < translatedLeftPulseWidth || 1 < translatedRightPulseWidth)
//		{
//			if (translatedLeftPulseWidth <= translatedRightPulseWidth)
//			{
//				translatedLeftPulseWidth = translatedLeftPulseWidth/translatedRightPulseWidth; 
//				translatedRightPulseWidth = 1;
//			}
//			else
//			{
//				translatedRightPulseWidth = translatedRightPulseWidth/translatedLeftPulseWidth; 
//				translatedLeftPulseWidth = 1;
//			}
//		}
		
		if (0 < power[L])
		{
			// 16384 + 8000 + (scaled) power[L]: increase left motor velocity.
			power[L] = PMS5005.PWM_N + PMS5005.PWM_O + power[L]*(PMS5005.DUTY_CYCLE_UNIT/3.0);
		}
		else if (power[L] < 0)
		{
			// 16384 - 8000 + (scaled) power[L]: reduce left motor velocity.
			power[L] = PMS5005.PWM_N - PMS5005.PWM_O - power[L]*(PMS5005.DUTY_CYCLE_UNIT/3.0); 
		}
		else
		{
			// neutral PWM setting, 0% duty cycle, let left motor sit idle
			power[L] = PMS5005.PWM_N;
		}
		
		//if (power[L] > L_MAX_PWM)
		//{
		// L_MAX_PWM = 32767, +100% duty cycle
		//	power[L] = L_MAX_PWM;
		//}
		//else if (power[L] < N_PWM)
		//{
		// stand still, 0% duty cycle
		//	power[L] = N_PWM;
		//}
		
		if (0 < power[R])
		{
			power[R] = PMS5005.PWM_N - PMS5005.PWM_O - power[R]*(PMS5005.DUTY_CYCLE_UNIT/3.0); // reverse: 16384 - 8000 - power[R]
		}
		else if (power[R] < 0)
		{
			power[R] = PMS5005.PWM_N + PMS5005.PWM_O + power[R]*(PMS5005.DUTY_CYCLE_UNIT/3.0); // reverse: 16384 + 8000 + rightPulseWidth
		}
		else
		{
			power[R] = PMS5005.PWM_N; // neutral PWM setting, 0% duty cycle
		}
		//if (power[R] < R_MAX_PWM) power[R] = R_MAX_PWM; // yes, < .. negative threshold @ -100% duty cycle
		//else if (power[R] > N_PWM) power[R] = N_PWM; // stand still, 0% duty cycle
		//robot.dcMotorPwmNonTimeCtrlAll((int)power[L], (int)power[R], NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
		
		System.err.println("translatedLeftPulseWidth: " + power[L]);
		System.err.println("translatedRightPulseWidth: " + power[R]);
		
		socket.send(PMS5005.setAllDCMotorPulses((short) power[L], (short) power[R], (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	//} else Robot.DcMotorPwmNonTimeCtrAll((short)N_PWM, (short)N_PWM, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL); Sleep(MICRO_DELAY);
	}
}
