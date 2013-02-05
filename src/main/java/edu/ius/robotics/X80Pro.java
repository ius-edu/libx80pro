package edu.ius.robotics;

import edu.ius.robotics.sockets.X80ProUdpSocket;
import edu.ius.robotics.boards.PMS5005;

public class X80Pro implements IX80Pro
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
	public static final int SERVO0_INI = 3650;
	public static final int SERVO1_INI = 3300;
	
	// private ByteArrayInputStream byteArrIn;
	// private DataInputStream dataIn;
	// private DataOutputStream dataOut;
	// private InputStream inputStream;
	// private BufferedReader reader;
	// private String command;
	/** wheel distance */
	final double WHEEL_CIR = 0.265;
	
	/** wheel radius */
	final double WHEEL_RADIUS = 0.0825;
	
	/** encoder one circle count */
	final int CIRCLE_CNT = 1200;
	
	private X80ProUdpSocket socket;
	
	/**
	 * Creates a new instance of X80Pro
	 * 
	 * @param robotIp
	 * @param robotPort
	 */
	public X80Pro(String ipAddress)
	{
		this.socket = new X80ProUdpSocket(ipAddress, DEFAULT_ROBOT_PORT);
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
		this.socket = new X80ProUdpSocket(ipAddress, port);
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
	 * analog to digital conversion
	 * 
	 * @param AdValue
	 * @return
	 */
	public static double Ad2Dis(int AdValue)
	{
		double distance = 0;
		
		if (AdValue > 4095 || AdValue <= 0)
		{
			distance = 0;
		}
		else
		{
			distance = 21.6 / ((double) (AdValue) * 3.0 / 4028 - 0.17);
			
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
		setAllServoPulses(SERVO0_INI, SERVO1_INI, NO_CTRL, NO_CTRL, NO_CTRL, NO_CTRL);
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
		return PMS5005.getSensorSonar((byte) channel, socket.getStandardSensorAry());
	}
	
	public int getSensorIRRange(int channel)
	{
		return PMS5005.getSensorIRRange((byte) channel, socket.getStandardSensorAry(), socket.getCustomSensorAry());
	}
	
	public int getSensorHumanAlarm(int channel)
	{
		return PMS5005.getSensorHumanAlarm((byte) channel, socket.getStandardSensorAry());
	}
	
	public int getSensorHumanMotion(int channel)
	{
		return PMS5005.getSensorHumanMotion((byte) channel, socket.getStandardSensorAry());
	}
	
	public int getSensorTiltingX(int channel)
	{
		return PMS5005.getSensorTiltingX(socket.getStandardSensorAry());
	}
	
	public int getSensorTiltingY(int channel)
	{
		return PMS5005.getSensorTiltingY(socket.getStandardSensorAry());
	}
	
	public int getSensorOverheat(int channel)
	{
		return PMS5005.getSensorOverheat((byte) channel, socket.getStandardSensorAry());
	}
	
	public int getSensorTemperature()
	{
		return PMS5005.getSensorTemperature(socket.getStandardSensorAry());
	}
	
	public int getSensorIRCode(int index)
	{
		return PMS5005.getSensorIRCode((byte) index, socket.getStandardSensorAry());
	}
	
	public void setInfraredControlOutput(int lowWord, int highWord)
	{
		PMS5005.setInfraredControlOutput((short) lowWord, (short) highWord);
	}
	
	public int getSensorBatteryAD(int channel)
	{
		return PMS5005.getSensorBatteryAD((short) channel, socket.getStandardSensorAry());
	}
	
	public int getSensorRefVoltage()
	{
		return PMS5005.getSensorRefVoltage(socket.getStandardSensorAry());
	}
	
	public int getSensorPotVoltage()
	{
		return PMS5005.getSensorPotVoltage(socket.getStandardSensorAry());
	}
	
	public int getSensorPot(int channel)
	{
		return PMS5005.getSensorPot((byte) channel, socket.getMotorSensorAry());
	}
	
	public int getMotorCurrent(int channel)
	{
		return PMS5005.getMotorCurrent((byte) channel, socket.getMotorSensorAry());
	}
	
	public int getEncoderDirection(int channel)
	{
		return PMS5005.getEncoderDirection((byte) channel, socket.getMotorSensorAry());
	}
	
	public int getEncoderPulse(int channel)
	{
		return PMS5005.getEncoderPulse((byte) channel, socket.getMotorSensorAry());
	}
	
	public int getEncoderSpeed(int channel)
	{
		return PMS5005.getEncoderSpeed((byte) channel, socket.getMotorSensorAry());
	}
	
	public int getCustomAD(int channel)
	{
		return PMS5005.getCustomAD((byte) channel, socket.getCustomSensorAry());
	}
	
	public int getCustomDIn(int channel)
	{
		return PMS5005.getCustomDIn((byte) channel, socket.getCustomSensorAry());
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