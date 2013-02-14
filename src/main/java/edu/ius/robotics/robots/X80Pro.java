package edu.ius.robotics.robots;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.sockets.UDPSocket;

public class X80Pro implements IX80Pro, IRobot
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
	
	public static final int HEADER_SIZE = 7;
	public static final int PAYLOAD_OFFSET = 6;
	public static final int DID_OFFSET = 4;
	
	public static final int DEFAULT_MOTOR_SENSOR_TIME_PERIOD = 50;
	public static final int DEFAULT_STANDARD_SENSOR_TIME_PERIOD = 50;
	public static final int DEFAULT_CUSTOM_SENSOR_TIME_PERIOD = 50;
	
	public static final int ACCEPTABLE_ENCODER_DEVIATION = 16;
	public static final int NO_CONTROL = -32768;
	public static final int MODE_PWM = 0;
	public static final int MODE_POSITION = 1;
	public static final int MODE_VELOCITY = 2;

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

	public static final int L_MAX_PWM = 32767;
	public static final int R_MAX_PWM = 0;
	public static final int N_PWM = 16383;
	public static final int O_PWM = 8000;
	public static final int DUTY_CYCLE_UNIT = 8383;

	public static final int MICRO_DELAY = 50;
	public static final int MINI_DELAY = 100;
	public static final int DELAY = 500;
	public static final int BIG_DELAY = 1000;
	public static final int UBER_DELAY = 5000;

	public static final short WHEEL_ENCODER_2PI = 800;

	public static final short MAX_IR = 81;
	public static final short MIN_IR = 9;

	public static final int L = 0;
	public static final int R = 1;

	public static final int NUM_IR_SENSORS = 7;
	public static final int NUM_IR_SENSORS_FRONT = 4;
	public static final int MAX_SPEED = 32767;
	public static final int RANGE = 30;
	
	// private ByteArrayInputStream byteArrIn;
	// private DataInputStream dataIn;
	// private DataOutputStream dataOut;
	// private InputStream inputStream;
	// private BufferedReader reader;
	// private String command;
	/** wheel distance */
	public static final double WHEEL_CIR = 0.265;
	public static final double WHEEL_DISPLACEMENT = 2.0; // TODO: Fix this (incorrect value)

	/** wheel radius */
	public static final double WHEEL_RADIUS = 0.0825;
	
	/** encoder one circle count */
	public static final int CIRCLE_CNT = 1200;
	
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
	
	private int[] motorSensorData;
	private int[] standardSensorData;
	private int[] customSensorData;
	//private int[] powerSensorData;
	
	private UDPSocket socket;
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param robotIp
	 * @param robotPort
	 */
	public X80Pro(String ipAddress)
	{
//		this.motorSensorTimePeriod = DEFAULT_MOTOR_SENSOR_TIME_PERIOD;
//		this.standardSensorTimePeriod = DEFAULT_STANDARD_SENSOR_TIME_PERIOD;
//		this.customSensorTimePeriod = DEFAULT_CUSTOM_SENSOR_TIME_PERIOD;
		
		this.motorSensorData = new int[34];
		this.customSensorData = new int[38];
		this.standardSensorData = new int[41];
		//this.powerSensorData = new int[32];
		
		this.socket = new UDPSocket(this, ipAddress, DEFAULT_ROBOT_PORT);
		//if (!socket.connectRobot(ipAddress, DEFAULT_ROBOT_PORT, MIN_TIME_MILLI)) System.exit(3);
	}
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param robotIp
	 * @param robotPort
	 */
	public X80Pro(String ipAddress, int port)
	{
//		this.motorSensorTimePeriod = DEFAULT_MOTOR_SENSOR_TIME_PERIOD;
//		this.standardSensorTimePeriod = DEFAULT_STANDARD_SENSOR_TIME_PERIOD;
//		this.customSensorTimePeriod = DEFAULT_CUSTOM_SENSOR_TIME_PERIOD;
		
		this.motorSensorData = new int[34];
		this.customSensorData = new int[38];
		this.standardSensorData = new int[41];
		//this.powerSensorData = new int[32];
		
		this.socket = new UDPSocket(this, ipAddress, port);
		//if (!socket.connectRobot(ipAddress, robotPort, MIN_TIME_MILLI)) System.exit(3);
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
	
	public void sensorEvent(int[] sensorData)
	{
		int z = sensorData.length;
		
		if (sensorData[0] == PMS5005.STX0 && sensorData[1] == PMS5005.STX1 && sensorData[z-2] == PMS5005.ETX0
				&& sensorData[z-1] == PMS5005.ETX1)
		{
			// here is a whole package (all sensor data)
			// for first motor sensor data, please refer to the protocol
			// documentation
			if (sensorData[DID_OFFSET] == PMS5005.GET_MOTOR_SENSOR_DATA)
			{
				this.motorSensorData = sensorData;
			}
			else if (sensorData[DID_OFFSET] == PMS5005.GET_CUSTOM_SENSOR_DATA)
			{
				this.customSensorData = sensorData;
			}
			else if (sensorData[DID_OFFSET] == PMS5005.GET_STANDARD_SENSOR_DATA)
			{
				this.motorSensorData = sensorData;
			}
		}
	}
	
	public void shutdown()
	{
		System.err.println("Shutting down robot");
		socket.setFinished(true);
	}
	
	/**
	 * analog to digital conversion
	 * 
	 * @param ADValue
	 * @return
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
		socket.send(PMS5005.enableAllSensorSending());
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
	
	public int getSensorSonar(int channel)
	{
		return (byte) (standardSensorData[channel + PMS5005.ULTRASONIC_OFFSET] & 0xff);
	}
	
	public int getSensorIRRange(int channel)
	{
		short result = -1;
		
		if (0 <= channel && channel < 1)
		{
			result = (short) (((standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET + 1] & 0xff) << 8) | (standardSensorData[PMS5005.STANDARD_IR_RANGE_OFFSET] & 0xff));
		}
		else
		{
			result = (short) (((customSensorData[2 * (channel - 1) + PMS5005.CUSTOM_IR_RANGE_OFFSET + 1] & 0xff) << 8) | (customSensorData[2 * (channel - 1) + PMS5005.CUSTOM_IR_RANGE_OFFSET] & 0xff));
		}
		
		return result;
	}
	
	public int getSensorHumanAlarm(int channel)
	{
		int offset = 2 * channel + PMS5005.HUMAN_ALARM_OFFSET;
		return (short) (((standardSensorData[offset + 1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getSensorHumanMotion(int channel)
	{
		int offset = 2 * channel + PMS5005.HUMAN_MOTION_OFFSET;
		return (short) (((standardSensorData[offset + 1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getSensorTiltingX(int channel)
	{
		return (short) (((standardSensorData[PMS5005.TILTING_X_OFFSET + 1] & 0xff) << 8) | (standardSensorData[PMS5005.TILTING_X_OFFSET] & 0xff));
	}
	
	public int getSensorTiltingY(int channel)
	{
		return (short) (((standardSensorData[PMS5005.TILTING_Y_OFFSET + 1] & 0xff) << 8) | (standardSensorData[PMS5005.TILTING_Y_OFFSET] & 0xff));
	}
	
	public int getSensorOverheat(int channel)
	{
		int offset = 2 * channel + PMS5005.OVERHEAT_SENSOR_OFFSET;
		return (short) (((standardSensorData[offset + 1] & 0xff) << 8) | (standardSensorData[offset] & 0xff));
	}
	
	public int getSensorTemperature()
	{
		return (short) (((standardSensorData[PMS5005.TEMPERATURE_AD_OFFSET + 1] & 0xff) << 8) | (standardSensorData[PMS5005.TEMPERATURE_AD_OFFSET] & 0xff));
	}
	
	public int getSensorIRCode(int index)
	{
		return (short) standardSensorData[PMS5005.INFRARED_COMMAND_OFFSET + index];
	}
	
	public void setInfraredControlOutput(int lowWord, int highWord)
	{
		//PMS5005.setInfraredControlOutput((short) lowWord, (short) highWord);
	}
	
	public int getSensorBatteryAD(int channel)
	{
		return (short) (((standardSensorData[2 * channel + PMS5005.BATTERY_SENSOR_OFFSET + 1] & 0xff) << 8 | standardSensorData[2 * channel + PMS5005.BATTERY_SENSOR_OFFSET] & 0xff));

	}
	
	public int getSensorRefVoltage()
	{
		return (short) (((standardSensorData[PMS5005.REFERENCE_VOLTAGE_OFFSET + 1] & 0xff) << 8 | standardSensorData[PMS5005.REFERENCE_VOLTAGE_OFFSET] & 0xff));
	}
	
	public int getSensorPotVoltage()
	{
		return (short) (((standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET + 1] & 0xff) << 8 | standardSensorData[PMS5005.POTENTIOMETER_POWER_OFFSET] & 0xff));
	}
	
	public int getSensorPot(int channel)
	{
		return (short) (((motorSensorData[2 * channel + PMS5005.POTENTIOMETER_SENSOR_OFFSET] + 1) << 8 | motorSensorData[2 * channel + PMS5005.POTENTIOMETER_SENSOR_OFFSET]));

	}
	
	public int getMotorCurrent(int channel)
	{
		return (short) ((((motorSensorData[2 * channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET + 1]) << 8 | motorSensorData[2 * channel + PMS5005.MOTOR_CURRENT_SENSOR_OFFSET])) / 728.0);
	}
	
	public int getEncoderDirection(int channel)
	{
		int offset = channel + PMS5005.ENCODER_DIRECTION_OFFSET;
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
		int offset = 4 * channel + PMS5005.ENCODER_PULSE_OFFSET;
		return (short) (((motorSensorData[offset + 1] & 0xff) << 8) | (motorSensorData[offset] & 0xff));
	}
	
	public int getEncoderSpeed(int channel)
	{
		int offset = 4 * channel + PMS5005.MOTOR_SPEED_OFFSET;
		return (short) (((motorSensorData[offset + 1] & 0xff) << 8) | (motorSensorData[offset] & 0xff));
	}
	
	public int getCustomAD(int channel)
	{
		int offset = 2 * channel + PMS5005.CUSTOM_AD_OFFSET;
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
	
	public void setDCMotorControlMode(int channel, int controlMode)
	{
		socket.send(PMS5005.setDCMotorControlMode((byte) channel, (byte) controlMode));
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
		socket.send(PMS5005.setLCDDisplayPMS(bmpFileName));
	}
	
	public void setDCMotorVelocityControlPID(byte channel, int kp, int kd, int ki)
	{
		socket.send(PMS5005.setDCMotorVelocityControlPID(channel, kp, kd, ki));
	}

	public void setBothDCMotorPositions(int pos0, int pos1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) pos1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorPositions(int pos0, int pos1) 
	{
		socket.send(PMS5005.setAllDCMotorPositions((short) pos0, (short) pos1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothDCMotorVelocities(int v0, int v1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) v1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorVelocities(int v0, int v1) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) v0, (short) v1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothDCMotorPulses(int p0, int p1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorVelocities((short) p0, (short) p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothDCMotorPulses(int p0, int p1) 
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}

	public void setBothServoPulses(int p0, int p1, int timePeriod) 
	{
		socket.send(PMS5005.setAllDCMotorPulses((short) p0, (short) p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) timePeriod));
	}

	public void setBothServoPulses(int p0, int p1) 
	{
		socket.send(PMS5005.setAllServoPulses((short) p0, (short) p1, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL, (short) NO_CTRL));
	}
}