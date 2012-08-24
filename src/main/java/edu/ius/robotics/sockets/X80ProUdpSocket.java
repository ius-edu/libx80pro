package edu.ius.robotics.sockets;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import edu.ius.robotics.boards.PMS5005;

public class X80ProUdpSocket implements Runnable
{
	private DatagramSocket socket;
	private String robotIp;
	private int robotPort;
	private DatagramPacket rxPkt;
	private DatagramPacket txPkt;
	private InetAddress server;
	private byte[] rxBuf;
	private byte[] txBuf;
	
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
	
	private int[] motorSensorAry;
	private int[] standardSensorAry;
	private int[] customSensorAry;
	
	public static final int HEADER_SIZE = 7;
	public static final int PAYLOAD_OFFSET = 6;
	public static final int DID_OFFSET = 4;
	public static final int DEFAULT_MIN_TIME_STEP = 50;
	public static final int DEFAULT_ROBOT_PORT = 10001;
	
	private int minTimeInMillis;
	
	public X80ProUdpSocket(String ipAddress)
	{
		connectRobot(ipAddress, DEFAULT_ROBOT_PORT, DEFAULT_MIN_TIME_STEP);
	}
	
	public X80ProUdpSocket(String ipAddress, int port)
	{
		connectRobot(ipAddress, port, DEFAULT_MIN_TIME_STEP);
	}
	
	public X80ProUdpSocket(String ipAddress, int port, int minTimeInMillis)
	{
		connectRobot(ipAddress, port, minTimeInMillis);
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
	
	public boolean connectRobot(String robotIp, int robotPort, int minTimeSlice)
	{
		boolean result;
		
		System.err.println("robotIp = " + robotIp);
		System.err.println("robotPort = " + robotPort);
		System.err.println("minTimeSlice = " + minTimeSlice);
		
		try
		{
			this.server = InetAddress.getByName(robotIp);
			this.robotIp = robotIp; 
			this.robotPort = robotPort;
			
			try
			{
				// socket connect
				socket = new DatagramSocket();
				socket.setSoTimeout(minTimeSlice);
			}
			catch (IOException ex)
			{
				ex.printStackTrace();
				result = false;
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
			
			result = true;
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			result = false;
		}
		
		return result;
	}
	
	public void send(byte[] cmd) 
	{
		System.arraycopy(cmd, 0, txBuf, 0, cmd.length);
		txPkt = new DatagramPacket(txBuf, txBuf.length, server, robotPort);
		
		try {
			this.socket.send(txPkt);
		} catch(IOException io)	{
			io.printStackTrace();
		}
	}
	
	/**
	 * receive packet thread
	 */
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
			
			splitSensorData(sensorDataAry);
			rxPkt.setLength(rxBuf.length);
		}	
	}
	
	private void splitSensorData(int[] sensorDataAry)
	{
		int z = sensorDataAry.length;
		
		if (sensorDataAry[0] == PMS5005.STX0 && sensorDataAry[1] == PMS5005.STX1 && sensorDataAry[z - 2] == PMS5005.ETX0
				&& sensorDataAry[z - 1] == PMS5005.ETX1)
		{
			// here is a whole package (all sensor data)
			// for first motor sensor data, please refer to the protocol
			// documentation
			if (sensorDataAry[DID_OFFSET] == PMS5005.GET_MOTOR_SENSOR_DATA)
			{
				setMotorSensorAry(sensorDataAry);
			}
			else if (sensorDataAry[DID_OFFSET] == PMS5005.GET_CUSTOM_SENSOR_DATA)
			{
				setCustomSensorAry(sensorDataAry);
			}
			else if (sensorDataAry[DID_OFFSET] == PMS5005.GET_STANDARD_SENSOR_DATA)
			{
				setStandardSensorAry(sensorDataAry);
			}
		}
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
	
	public int getMinTimeInMillis()
	{
		return minTimeInMillis;
	}

	public void setMinTimeInMillis(int minTimeInMillis)
	{
		this.minTimeInMillis = minTimeInMillis;
	}

	public int[] getMotorSensorAry()
	{
		return motorSensorAry;
	}

	public void setMotorSensorAry(int[] motorSensorAry)
	{
		this.motorSensorAry = motorSensorAry;
	}

	public int[] getStandardSensorAry()
	{
		return standardSensorAry;
	}

	public void setStandardSensorAry(int[] standardSensorAry)
	{
		this.standardSensorAry = standardSensorAry;
	}

	public int[] getCustomSensorAry()
	{
		return customSensorAry;
	}

	public void setCustomSensorAry(int[] customSensorAry)
	{
		this.customSensorAry = customSensorAry;
	}
}
