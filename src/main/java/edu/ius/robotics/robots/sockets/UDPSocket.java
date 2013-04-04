package edu.ius.robotics.robots.sockets;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
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
		return this.port;
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
		this.delay = DEFAULT_DELAY;
		
		boolean result;
		
		try
		{
			this.server = InetAddress.getByName(this.ip);
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
		
		this.server = InetAddress.getByName(ip);
		this.ip = ip; 
		this.port = port;
		
		this.socket = new DatagramSocket();
		this.socket.setSoTimeout(delay);

		this.rxBuf = new byte[4097];
		this.rxPkt = new DatagramPacket(this.rxBuf, this.rxBuf.length);
		
		//this.txBuf = new byte[256];
		this.txBuf = new byte[65];
		this.txPkt = new DatagramPacket(this.txBuf, this.txBuf.length, this.server, this.port);
		
		this.socket.send(this.txPkt);
		
		// wait synchronously for feedback from robot to be clear that we have established connection
		//this.socket.setSoTimeout(CONNECT_WAIT);
		//this.socket.receive(this.rxPkt);

		new Thread(this).start(); // receive packet thread
	}
	
	public void send(byte[] cmd) 
	{
		long beginTime = System.currentTimeMillis();
		System.arraycopy(cmd, 0, this.txBuf, 0, cmd.length);
		this.txPkt = new DatagramPacket(this.txBuf, this.txBuf.length, this.server, this.port);
		try
		{
			this.socket.send(this.txPkt);
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
		long nextUsableTick = beginTime + this.delay;
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
		while (!this.isFinished)
		{
			//long beginTime = System.currentTimeMillis();
			try
			{
				this.socket.setSoTimeout(10);
				this.socket.receive(this.rxPkt);
			}
			catch (IOException ex)
			{
				//ex.printStackTrace();
				//System.err.println("did not receive sensor data");
			}
			//int z = this.rxPkt.getLength();
			if (0 < this.rxPkt.getLength()) // 0 < z
			{
				// decode here
				//byte[] sensorData = new byte[z];
				//System.err.println("DEBUG: raw packet: ");
				//for (int i = 0; i < z; ++i)
				//{
					//sensorData[i] = (byte) (this.rxBuf[i] & 0xFF);
					//System.err.print("DEBUG: " + sensorData[i] + " ");
				//}
				//System.err.println("DEBUG: ");
				// callback
				this.iRobot.sensorEvent(this.ip, this.port, this.rxBuf, this.rxPkt.getLength());
				//this.rxPkt.setLength(this.rxBuf.length);
			}
			//System.out.println("sensorEvent took " + (System.currentTimeMillis() - beginTime) + "ms to complete");
		}
	}
}
