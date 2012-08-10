package edu.ius.robotics;

import java.io.*;
import java.net.*;

/**
 * 
 * @author Bo Wei modifications by Jesse Riddle
 */
public class X80 implements IX80, Runnable
{
	/** minimum time step in milliseconds */
	public final int minTimeSlice = 50;
	
	public final int nStdSensors = 6;
	public final int nMotors = 2;
	public final int nIrSensors = 8; 
	public final int nCustomSensors = 9;
	public final int nUltrasonicSensors = 6;
	public final int nHumanSensors = 2;
	
	private DatagramSocket socket;
	private String robotIp;
	private int robotPort;
	private DatagramPacket rxPkt;
	private DatagramPacket txPkt;
	private InetAddress server;
	private byte[] rxBuf;
	private byte[] txBuf;
	// private ByteArrayInputStream byteArrIn;
	// private DataInputStream dataIn;
	// private DataOutputStream dataOut;
	// private InputStream inputStream;
	// private BufferedReader reader;
	// private String command;
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
	public int[] humanAlarm;
	public int[] humanMotion;
	public int irRange;
	public double boardVol;
	public double dcMotorVol;
	
	/** wheel distance */
    final double WheelDis = 0.265;
    
    /** wheel radius */
    final double WheelR = 0.0825;
    
    /** encoder one circle count */
    final int CircleCnt = 1200;
	
	/** Creates a new instance of X80 */
	public X80(String robotIp, int robotPort)
	{
		this.makeSensorData();
		this.clearSensorData();
		
		if (this.connectRobot(robotIp, robotPort))
		{
			try
			{
				// socket connect
				socket = new DatagramSocket();
				socket.setSoTimeout(minTimeSlice);
			}
			catch (IOException ex)
			{
				ex.printStackTrace();
				System.exit(3);
				return;
			}
			
			rxBuf = new byte[1024];
			rxPkt = new DatagramPacket(rxBuf, rxBuf.length);
			
			txBuf = new byte[256];
			txPkt = new DatagramPacket(txBuf, txBuf.length, server, robotPort);
			
			try
			{
				socket.send(txPkt);
			}
			catch (IOException ex)
			{
				ex.printStackTrace();
			}
		}
		else
		{
			System.exit(3);
		}
	}
	
	private void makeSensorData()
	{
		this.encoderPos = new int[nMotors];
		this.irDis = new double[nIrSensors];
		this.encoderSpeed = new int[nMotors];
		this.motorCurrent = new double[nMotors];
		this.customAd = new int[nCustomSensors];
		this.usDis = new double[nUltrasonicSensors];
		this.humanAlarm = new int[nHumanSensors];
		this.humanMotion = new int[nHumanSensors];
	}
	
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
	
	public void run()
	{
		try
		{
			socket.setSoTimeout(10);
			socket.receive(rxPkt);
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
		
		int z = rxPkt.getLength();
		
		if (0 < z)
		{
			// decode here
			int[] sensorDataAry = new int[z];
			
			for (int i = 0; i < z; ++i)
			{
				sensorDataAry[i] = (int) (rxBuf[i] & 0xff);
			}
			
			decodeSensorData(sensorDataAry);
			rxPkt.setLength(rxBuf.length);
		}
		
	}
	
	public void sendCommand(byte[] command)
	{
		System.arraycopy(command, 0, txBuf, 0, command.length);
		txPkt = new DatagramPacket(txBuf, txBuf.length, server, robotPort);
		
		try
		{
			socket.send(txPkt);
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
	}
	
	private double Ad2Dis(int AdValue)
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
	
	public String getRobotIp()
	{
		return robotIp;
	}
	
	public int getRobotPort()
	{
		return robotPort;
	}
	
	public void setRobotIp(String robotIp)
	{
		this.robotIp = robotIp;
	}
	
	public void setRobotPort(int robotPort)
	{
		this.robotPort = robotPort;
	}
	
	public boolean connectRobot()
	{
		boolean result;
		
		try
		{
			this.server = InetAddress.getByName(robotIp);
			result = true;
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			result = false;
		}
		
		return result;
	}
	
	public boolean connectRobot(String robotIp, int robotPort)
	{
		boolean result;
		
		try
		{
			this.server = InetAddress.getByName(robotIp);
			this.setRobotIp(robotIp);
			this.setRobotPort(robotPort);
			result = true;
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			result = false;
		}
		
		return result;
	}
	
	private void decodeSensorData(int[] sensorDataAry)
	{
		int z = sensorDataAry.length;
		
		if (sensorDataAry[0] == 94 && sensorDataAry[1] == 2 && sensorDataAry[z - 2] == 94 && sensorDataAry[z - 1] == 13)
		{
			// here is a whole package (all sensor data)
			// for first motor sensor data, please refer to the protocol
			// documentation
			if (sensorDataAry[4] == 123)
			{
				encoderPos[0] = sensorDataAry[5 + 25] + (sensorDataAry[5 + 26]) * 256;
				encoderSpeed[0] = sensorDataAry[5 + 27] + (sensorDataAry[5 + 28]) * 256;
				encoderPos[1] = sensorDataAry[5 + 29] + (sensorDataAry[5 + 30]) * 256;
				encoderSpeed[1] = sensorDataAry[5 + 31] + (sensorDataAry[5 + 32]) * 256;
				
				motorCurrent[0] = (double) (sensorDataAry[5 + 13] + (sensorDataAry[5 + 14]) * 256) / 728.0;
				motorCurrent[1] = (double) (sensorDataAry[5 + 15] + (sensorDataAry[5 + 16]) * 256) / 728.0;
			}
			else if (sensorDataAry[4] == 124)
			{
				// custom sensor data
				for (int i = 0; i < 8; i++)
				{
					customAd[i] = sensorDataAry[6 + 2 * i] + sensorDataAry[6 + 2 * i + 1] * 256;
				}
				
				customIo = sensorDataAry[5 + 17];
			}
			else if (sensorDataAry[4] == 125)
			{
				// standard sensors
				for (int i = 0; i < nStdSensors; ++i)
				{
					usDis[i] = (double) (sensorDataAry[nStdSensors + i]) / 100.0;
				}
				
				humanAlarm[0] = sensorDataAry[5 + 7] + sensorDataAry[5 + 8] * 256;
				humanMotion[0] = sensorDataAry[5 + 9] + sensorDataAry[5 + 10] * 256;
				humanAlarm[1] = sensorDataAry[5 + 11] + sensorDataAry[5 + 12] * 256;
				humanMotion[1] = sensorDataAry[5 + 13] + sensorDataAry[5 + 14] * 256;
				irRange = (sensorDataAry[5 + 25] + sensorDataAry[5 + 26] * 256);
				boardVol = (double) (sensorDataAry[5 + 31] + sensorDataAry[5 + 32] * 256) / 4095.0 * 9.0;
				dcMotorVol = (double) (sensorDataAry[5 + 33] + sensorDataAry[5 + 34] * 256) / 4095.0 * 24.0;
			}
		}
		
		// the first ir sensor
		irDis[0] = Ad2Dis(irRange);
		
		// the rest of the ir sensors
		for (int i = 0; i < nIrSensors; i++)
		{
			irDis[i + 1] = Ad2Dis(customAd[i + 1]);
		}
	}
	
	public void motorSensorRequest(int packetNumber)
	{
		sendCommand(PMS5005.motorSensorRequest(packetNumber));
	}

	public void standardSensorRequest(int packetNumber)
	{
		sendCommand(PMS5005.standardSensorRequest(packetNumber));
	}

	public void customSensorRequest(int packetNumber)
	{
		sendCommand(PMS5005.customSensorRequest(packetNumber));
	}

	public void allSensorRequest(int packetNumber)
	{
		sendCommand(PMS5005.allSensorRequest(packetNumber));
	}

	public void enableMotorSensorSending()
	{
		sendCommand(PMS5005.enableAllSensorSending());
	}

	public void enableStandardSensorSending()
	{
		sendCommand(PMS5005.enableStandardSensorSending());
	}

	public void enableCustomSensorSending()
	{
		sendCommand(PMS5005.enableCustomSensorSending());
	}

	public void enableAllSensorSending()
	{
		sendCommand(PMS5005.enableAllSensorSending());
	}

	public void disableMotorSensorSending()
	{
		sendCommand(PMS5005.disableMotorSensorSending());
	}

	public void disableStandardSensorSending()
	{
		sendCommand(PMS5005.disableStandardSensorSending());
	}

	public void disableCustomSensorSending()
	{
		sendCommand(PMS5005.disableCustomSensorSending());
	}

	public void disableAllSensorSending()
	{
		sendCommand(PMS5005.disableAllSensorSending());
	}

	public void setMotorSensorPeriod(int timePeriod)
	{
		sendCommand(PMS5005.setMotorSensorPeriod(timePeriod));
	}

	public void setStandardSensorPeriod(int timePeriod)
	{
		sendCommand(PMS5005.setStandardSensorPeriod(timePeriod));
	}

	public void setCustomSensorPeriod(int timePeriod)
	{
		sendCommand(PMS5005.setCustomSensorPeriod(timePeriod));
	}

	public void setAllSensorPeriod(int timePeriod)
	{
		sendCommand(PMS5005.setAllSensorPeriod(timePeriod));
	}

	public int getSensorSonar(int channel)
	{
		// TODO examine this farther
		return PMS5005.getSensorSonar(channel);
	}

	public int getSensorIrRange(int channel)
	{
		// TODO examine this farther
		return PMS5005.getSensorIrRange(channel);
	}

	public int getSensorHumanAlarm(int channel)
	{
		// TODO examine this farther
		return PMS5005.getSensorHumanAlarm(channel);
	}

	public int getSensorHumanMotion(int channel)
	{
		// TODO examine this farther
		return PMS5005.getSensorHumanMotion(channel);
	}

	public int getSensorTiltingX(int channel)
	{
		// TODO examine this farther
		return PMS5005.getSensorTiltingX(channel);
	}

	public int getSensorTiltingY(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorOverheat(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorTemperature()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorIrCode(int index)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public void setInfraredControlOutput(int lowWord, int highWord)
	{
		// TODO Auto-generated method stub
		
	}

	public int getSensorBatteryAd(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorRefVoltage()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorPotVoltage()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorPot(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getMotorCurrent(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderDir(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderPulse(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderSpeed(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getCustomAd(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getCustomDin(int channel)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public void setCustomDout(int ival)
	{
		// TODO Auto-generated method stub
		
	}

	public void setMotorPolarity(int channel, int polarity)
	{
		// TODO Auto-generated method stub
		
	}

	public void enableDcMotor(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void disableDcMotor(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void resumeDcMotor(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void suspendDcMotor(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorPositionControlPid(int channel, int Kp, int Kd, int Ki_x100)
	{
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorSensorFilter(int channel, int filterMethod)
	{
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorSensorUsage(int channel, int sensorType)
	{
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorControlMode(int channel, int controlMode)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionNonTimeCtrl(int channel, int cmdValue)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmNonTimeCtrl(int channel, int cmdValue)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorVelocityTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorVelocityNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		// TODO Auto-generated method stub
		
	}

	public void enableServo(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void disableServo(int channel)
	{
		// TODO Auto-generated method stub
		
	}

	public void servoTimeCtrl(int channel, int cmdValue, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void servoNonTimeCtrl(int channel, int cmdValue)
	{
		// TODO Auto-generated method stub
		
	}

	public void servoTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
	{
		// TODO Auto-generated method stub
		
	}

	public void servoNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
	{
		// TODO Auto-generated method stub
		
	}

	public void lcdDisplayPMS(String bmpFileName)
	{
		// TODO Auto-generated method stub
		
	}
	
	
}