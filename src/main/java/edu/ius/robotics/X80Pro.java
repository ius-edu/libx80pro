package edu.ius.robotics;

import edu.ius.robotics.sockets.X80ProUdpSocket;
import edu.ius.robotics.boards.PMS5005;

public class X80Pro implements IX80Pro, Runnable
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
	final double WheelDis = 0.265;
	
	/** wheel radius */
	final double WheelR = 0.0825;
	
	/** encoder one circle count */
	final int CircleCnt = 1200;
	
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
	 * receive packet thread
	 */
	public void run()
	{
		socket.run();
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
		servoNonTimeCtrlAll(SERVO0_INI, SERVO1_INI, NO_CTRL, NO_CTRL, NO_CTRL, NO_CTRL);
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
	
	public int getSensorIrRange(int channel)
	{
		return PMS5005.getSensorIrRange((byte) channel, socket.getStandardSensorAry(), socket.getCustomSensorAry());
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
	
	public int getSensorIrCode(int index)
	{
		return PMS5005.getSensorIrCode((byte) index, socket.getStandardSensorAry());
	}
	
	public void setInfraredControlOutput(int lowWord, int highWord)
	{
		PMS5005.setInfraredControlOutput((short) lowWord, (short) highWord);
	}
	
	public int getSensorBatteryAd(int channel)
	{
		return PMS5005.getSensorBatteryAd((short) channel, socket.getStandardSensorAry());
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
	
	public int getCustomAd(int channel)
	{
		return PMS5005.getCustomAd((byte) channel, socket.getCustomSensorAry());
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
	public void enableDcMotor(int channel)
	{
		socket.send(PMS5005.enableDcMotor((byte) channel));
	}
	
	@SuppressWarnings("deprecation")
	public void disableDcMotor(int channel)
	{
		socket.send(PMS5005.disableDcMotor((byte) channel));
	}
	
	public void resumeDcMotor(int channel)
	{
		socket.send(PMS5005.resumeDcMotor((byte) channel));
	}
	
	public void suspendDcMotor(int channel)
	{
		socket.send(PMS5005.suspendDcMotor((byte) channel));
	}
	
	public void setDcMotorPositionControlPid(int channel, int Kp, int Kd, int Ki_x100)
	{
		socket.send(PMS5005.setDcMotorPositionControlPid((byte) channel, (short) Kp, (short) Kd, (short) Ki_x100));
	}
	
	public void setDcMotorSensorFilter(int channel, int filterMethod)
	{
		socket.send(PMS5005.setDcMotorSensorFilter((byte) channel, (short) filterMethod));
	}
	
	public void setDcMotorSensorUsage(int channel, int sensorType)
	{
		socket.send(PMS5005.setDcMotorSensorUsage((byte) channel, (byte) sensorType));
	}
	
	public void setDcMotorControlMode(int channel, int controlMode)
	{
		socket.send(PMS5005.setDcMotorControlMode((byte) channel, (byte) controlMode));
	}
	
	public void dcMotorPositionTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		socket.send(PMS5005.dcMotorPositionTimeCtrl((byte) channel, (short) cmdValue, (short) timePeriod));
	}
	
	public void dcMotorPositionNonTimeCtrl(int channel, int cmdValue)
	{
		socket.send(PMS5005.dcMotorPositionNonTimeCtrl((byte) channel, (short) cmdValue));
	}
	
	public void dcMotorPwmTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		socket.send(PMS5005.dcMotorPwmTimeCtrl((byte) channel, (short) cmdValue, (short) timePeriod));
	}
	
	public void dcMotorPwmNonTimeCtrl(int channel, int cmdValue)
	{
		socket.send(PMS5005.dcMotorPwmNonTimeCtrl((byte) channel, (short) cmdValue));
	}
	
	public void dcMotorPositionTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		socket.send(PMS5005.dcMotorPositionTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6,
				(short) timePeriod));
	}
	
	public void dcMotorPositionNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		socket.send(PMS5005.dcMotorPositionNonTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6));
	}
	
	public void dcMotorVelocityTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		socket.send(PMS5005.dcMotorVelocityTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6,
				(short) timePeriod));
	}
	
	public void dcMotorVelocityNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		socket.send(PMS5005.dcMotorVelocityNonTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6));
	}
	
	public void dcMotorPwmTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		socket.send(PMS5005.dcMotorPwmTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6,
				(short) timePeriod));
	}
	
	public void dcMotorPwmNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		socket.send(PMS5005.dcMotorPwmNonTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6));
	}
	
	public void enableServo(int channel)
	{
		socket.send(PMS5005.enableServo((byte) channel));
	}
	
	public void disableServo(int channel)
	{
		socket.send(PMS5005.disableServo((byte) channel));
	}
	
	public void servoTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		socket.send(PMS5005.servoTimeCtrl((byte) channel, (short) cmdValue, (short) timePeriod));
	}
	
	public void servoNonTimeCtrl(int channel, int cmdValue)
	{
		socket.send(PMS5005.servoNonTimeCtrl((byte) channel, (short) cmdValue));
	}
	
	public void servoTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		socket.send(PMS5005.servoTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6, (short) timePeriod));
	}
	
	public void servoNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		socket.send(PMS5005.servoNonTimeCtrlAll((short) pos1, (short) pos2, (short) pos3, (short) pos4, (short) pos5, (short) pos6));
	}
	
	public void lcdDisplayPMS(String bmpFileName)
	{
		socket.send(PMS5005.lcdDisplayPMS(bmpFileName));
	}
	
	public void setDcMotorVelocityControlPID(byte channel, int Kp, int Kd, int Ki)
	{
		socket.send(PMS5005.setDcMotorVelocityControlPID(channel, Kp, Kd, Ki));
	}
}