package edu.ius.robotics.robots.sockets;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.interfaces.IRobot;

public class UDPSocket implements Runnable
{
	volatile boolean isFinished = false;
	
	private IRobot iRobot;
	
	public static final int DEFAULT_PORT = 10001;
	public static final int DEFAULT_DELAY = 100;
	public static final int CONNECT_WAIT = 5000;
	
	private DatagramSocket socket;
	private String ip;
	private int port;
	private DatagramPacket rxPkt;
	private DatagramPacket txPkt;
	private InetAddress server;
	private byte[] rxBuf;
	private byte[] txBuf;
	
	private int delay;
	
	public UDPSocket(IRobot iRobot, String ipAddress) throws IOException
	{
		connect(iRobot, ipAddress, DEFAULT_PORT, DEFAULT_DELAY);
	}
	
	public UDPSocket(IRobot iRobot, String ipAddress, int port) throws IOException
	{
		connect(iRobot, ipAddress, port, DEFAULT_DELAY);
	}
	
	public UDPSocket(IRobot robot, String ipAddress, int port, int delay) throws IOException
	{
		connect(iRobot, ipAddress, port, delay);
	}
	
	public String getip()
	{
		return this.ip;
	}
	
	public int getPort()
	{
		return port;
	}
	
	public void setip(String ip)
	{
		this.ip = ip;
	}
	
	public void setPort(int port)
	{
		this.port = port;
	}
	
	public boolean connect(IRobot iRobot)
	{
		this.iRobot = iRobot;
		delay = DEFAULT_DELAY;
		
		boolean result;
		
		try
		{
			server = InetAddress.getByName(ip);
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
	
	public void connect(IRobot iRobot, String ip, int port, int delay) throws IOException
	{
		this.iRobot = iRobot;
		this.delay = delay;
		
		System.err.println("ip = " + ip);
		System.err.println("port = " + port);
		System.err.println("delay = " + delay);
		
		server = InetAddress.getByName(ip);
		this.ip = ip; 
		this.port = port;
		
		socket = new DatagramSocket();
		socket.setSoTimeout(delay);
		
		rxBuf = new byte[8192];
		rxPkt = new DatagramPacket(rxBuf, rxBuf.length);
		
		txBuf = new byte[1024];
		txPkt = new DatagramPacket(txBuf, txBuf.length, server, port);
		
		byte[] sendBuffer = new byte[1024];
		int dataLength = PMS5005.ping(sendBuffer);
		send(sendBuffer, dataLength);
		// wait synchronously for feedback from robot to be clear that we have established connection
		socket.setSoTimeout(CONNECT_WAIT);
		socket.receive(rxPkt);

		new Thread(this).start(); // receive packet thread
	}
	
	public void send(byte[] data, int dataLength) 
	{
		long beginTime = System.currentTimeMillis();
		System.arraycopy(data, 0, txBuf, 0, dataLength);
		txPkt = new DatagramPacket(txBuf, dataLength, server, port);
		try
		{
			socket.send(txPkt);
		}
		catch (IOException ioException)	
		{
			ioException.printStackTrace();
		}
		delay(beginTime);
	}
	
	public void delay(long beginTime)
	{
		//int actionsPerSecond = 1000/DEFAULT_MIN_TIME_STEP_IN_MS;
		long nextUsableTick = beginTime + delay;
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
		isFinished = true;
		socket.close();
	}
	
	/**
	 * receive packet thread
	 */
	public void run() 
	{
		while (!this.isFinished) 
		{
			try 
			{
				socket.setSoTimeout(10);
				socket.receive(rxPkt);
				if (0 < rxPkt.getLength())
				{
					iRobot.socketEvent(ip, port, rxPkt.getData(), rxPkt.getLength());
				}
			}
			catch (IOException ex) 
			{
			}
		}
	}
}
