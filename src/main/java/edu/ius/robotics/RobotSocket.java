package edu.ius.robotics;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class RobotSocket
{
	private DatagramSocket socket;
	private String robotIp;
	private int robotPort;
	private DatagramPacket rxPkt;
	private DatagramPacket txPkt;
	private InetAddress server;
	private byte[] rxBuf;
	private byte[] txBuf;
	
	
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
}
