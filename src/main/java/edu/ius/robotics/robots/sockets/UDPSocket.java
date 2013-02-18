package edu.ius.robotics.robots.sockets;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import edu.ius.robotics.robots.IRobot;

public class UDPSocket implements Runnable
{
	volatile boolean isFinished = false;
	
	private IRobot iRobot;
	
	public static final int DEFAULT_ROBOT_PORT = 10001;
	public static final int DEFAULT_MIN_TIME_STEP_IN_MS = 60;
	public static final int CONNECT_WAIT = 5000;
	
	private DatagramSocket socket;
	private String robotIP;
	private int robotPort;
	private DatagramPacket rxPkt;
	private DatagramPacket txPkt;
	private InetAddress server;
	private byte[] rxBuf;
	private byte[] txBuf;
	
	private int minTimeInMS;
	
	public UDPSocket(IRobot iRobot, String ipAddress) throws IOException
	{
		connectRobot(iRobot, ipAddress, DEFAULT_ROBOT_PORT, DEFAULT_MIN_TIME_STEP_IN_MS);
	}
	
	public UDPSocket(IRobot iRobot, String ipAddress, int port) throws IOException
	{
		connectRobot(iRobot, ipAddress, port, DEFAULT_MIN_TIME_STEP_IN_MS);
	}
	
	public UDPSocket(IRobot robot, String ipAddress, int port, int minTimeInMS) throws IOException
	{
		connectRobot(iRobot, ipAddress, port, minTimeInMS);
	}
	
	public String getRobotIP()
	{
		return robotIP;
	}
	
	public int getRobotPort()
	{
		return robotPort;
	}
	
	public void setRobotIP(String robotIP)
	{
		this.robotIP = robotIP;
	}
	
	public void setRobotPort(int robotPort)
	{
		this.robotPort = robotPort;
	}
	
	public boolean connectRobot(IRobot iRobot)
	{
		this.iRobot = iRobot;
		this.minTimeInMS = DEFAULT_MIN_TIME_STEP_IN_MS;
		
		boolean result;
		
		try
		{
			this.server = InetAddress.getByName(robotIP);
			result = true;
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			result = false;
		}
		
		new Thread(this).start();
		return result;
	}
	
	//public boolean connectRobot(IRobot iRobot, String robotIP, int robotPort, int minTimeInMS) throws IOException
	public void connectRobot(IRobot iRobot, String robotIP, int robotPort, int minTimeInMS) throws IOException
	{
		this.iRobot = iRobot;
		this.minTimeInMS = minTimeInMS;
		
		//boolean result;
		
		System.err.println("robotIP = " + robotIP);
		System.err.println("robotPort = " + robotPort);
		System.err.println("minTimeInMS = " + minTimeInMS);
		
/*		try
		{
		*/
			this.server = InetAddress.getByName(robotIP);
			this.robotIP = robotIP; 
			this.robotPort = robotPort;
			
//			try
//			{
				socket = new DatagramSocket();
				socket.setSoTimeout(minTimeInMS);
//			}
//			catch (IOException ex)
//			{
//				ex.printStackTrace();
//				//result = false;
//			}
			
			rxBuf = new byte[1024];
			rxPkt = new DatagramPacket(rxBuf, rxBuf.length);
			
			txBuf = new byte[256];
			txPkt = new DatagramPacket(txBuf, txBuf.length, server, robotPort);
			
//			try
//			{
				socket.send(txPkt);
//			}
//			catch (IOException ex)
//			{
//				ex.printStackTrace();
//			}
			
			// wait synchronously for feedback from robot to be clear that we have established connection?
			socket.setSoTimeout(CONNECT_WAIT);
			socket.receive(rxPkt);
//			result = true;
//		}
//		catch (IOException ex)
//		{
//			ex.printStackTrace();
//			result = false;
//		}
//
//		if (result)
//		{
//			new Thread(this).start(); // receive packet thread
//		}
//		return result;
		new Thread(this).start(); // receive packet thread
	}
	
	public void send(byte[] cmd) 
	{
		long beginTime = System.currentTimeMillis();
		
		System.arraycopy(cmd, 0, txBuf, 0, cmd.length);
		txPkt = new DatagramPacket(txBuf, txBuf.length, server, robotPort);
		
		try
		{
			this.socket.send(txPkt);
			delay(beginTime);
		}
		catch (IOException ioException)	
		{
			ioException.printStackTrace();
		}
	}
	
	public void delay(long beginTime)
	{
		//int actionsPerSecond = 1000/DEFAULT_MIN_TIME_STEP_IN_MS;
		long nextUsableTick = beginTime + minTimeInMS;
		long sleepTime = nextUsableTick - System.currentTimeMillis();
		if (0 < sleepTime)
		{
			try
			{
				Thread.sleep(sleepTime);
			}
			catch (Exception ex)
			{
				ex.printStackTrace();
			}
		} // else: we are running behind
	}
	
	public void close()
	{
		this.isFinished = true;
		this.socket.close();
	}
	
	/**
	 * receive packet thread
	 */
	public void run()
	{
		System.err.println("UDPSocket.run(): receive thread");
		
		while (!isFinished)
		{
			try
			{
				socket.setSoTimeout(10);
				socket.receive(rxPkt);
			}
			catch (IOException ex)
			{
				//ex.printStackTrace();
				//System.err.println("did not receive sensor data");
			}
			
			int z = rxPkt.getLength();
			
			if (0 < z)
			{
				// decode here
				byte[] sensorData = new byte[z];
				
				//System.err.println("DEBUG: raw packet: ");
				for (int i = 0; i < z; ++i)
				{
					sensorData[i] = (byte) (rxBuf[i] & 0xff);
					//System.err.print("DEBUG: " + sensorData[i] + " ");
				}
				//System.err.println("DEBUG: ");
				
				// callback
				iRobot.sensorEvent(sensorData);
				rxPkt.setLength(rxBuf.length);
			}
		}
		
		System.err.println("UDPSocket.run(): terminated");
	}
}
