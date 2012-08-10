
package com.drrobot;

import java.io.*;
import java.net.*;

/**
 *
 * @author Bo Wei
 * modifications by Jesse Riddle
 */
public class RobotSocket implements Runnable
{
	public final int minTimeSlice = 20;
	
    private DatagramSocket socket;
    private String robotIp;
    private int robotPort;
    private DatagramPacket rxPkt;
    private DatagramPacket txPkt;
    private InetAddress server;
    private byte[] rxBuf;
    private byte[] txBuf;
    public int[] sensorDataAry;
    //private ByteArrayInputStream byteArrIn;
    //private DataInputStream dataIn;
    //private DataOutputStream dataOut;
    //private InputStream inputStream;
    //private BufferedReader reader;
    //private String command;
    public int[] encoderPos;
    public double[] irDis;
    //motor sensor 
    public int[] encoderSpeed;
    public double[] motorCurrent;
    //custom sensor data
    public int[] customAd;
    public int customIo;
    //standard sensor data
    public double[] usDis;
    public int[] humanAlarm;
    public int[] humanMotion;
    public int irRange;
    public double boardVol;
    public double dcMotorVol;
    
    /** Creates a new instance of robotSocket */
    public RobotSocket(String robotIp, int robotPort) 
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
    
    public void makeSensorData()
    {
    	this.encoderPos = new int[2];
    	this.irDis = new double[8];
    	this.encoderSpeed = new int[2];
    	this.motorCurrent = new double[2];
    	this.customAd = new int[9];
    	this.usDis = new double[6];
    	this.humanAlarm = new int[2];
    	this.humanMotion = new int[2];
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
    		this.irDis[c] = (double)(c + 1); 
    	}
    	
        //motor sensor 
    	for (c = 0, z = this.encoderSpeed.length; c < z; ++c)
    	{
    		this.encoderSpeed[c] = 0;
    	}
    	
    	for (c = 0, z = this.motorCurrent.length; c < z; ++c)
    	{
    		this.motorCurrent[c] = 0.0;
    	}

    	//custom sensor data
    	for (c = 0, z = this.customAd.length; c < z; ++c)
    	{
    		this.customAd[c] = 0;
    	}
    	
        this.customIo = 0;
        
        //standard sensor data
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
            //decode here
            sensorDataAry = new int[z];
            
            for (int i = 0; i < z; ++i) 
            {
                sensorDataAry[i] = (int)(rxBuf[i] & 0xff);
            }
            
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
}
