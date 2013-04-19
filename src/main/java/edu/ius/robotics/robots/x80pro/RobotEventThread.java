package edu.ius.robotics.robots.x80pro;

import java.io.ByteArrayOutputStream;
import java.util.concurrent.BlockingQueue;

import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;

public class RobotEventThread implements Runnable
{
	private boolean isReadyToDispatchEvents;
	private IRobotEventHandler iRobotEventHandler;
	private BlockingQueue eventQueue;
	
	public RobotEventThread(IRobotEventHandler iRobotEventHandler)
	{
		this.iRobotEventHandler = iRobotEventHandler;
	}
	
	public boolean getIsReadyToDispatchEvents()
	{
		return isReadyToDispatchEvents;
	}
	
	public void setIsReadyToDispatchEvents(boolean flag)
	{
		this.isReadyToDispatchEvents = flag;
	}
	
	public void dispatchSensorDataReceivedEvent(IRobot sender)
	{
		
	}
	
	public void dispatchAudioDataReceivedEvent(IRobot sender, short[] audioData)
	{
		
	}
	
	public void disaptchImageDataReceivedEvent(IRobot sender, ByteArrayOutputStream imageData)
	{
		
	}
	
	@Override
	public void run()
	{
		
		iRobotEventHandler.sensorDataReceivedEvent(sender, sensorDataType);
	}
}
