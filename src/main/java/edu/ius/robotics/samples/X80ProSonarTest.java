package edu.ius.robotics.samples;

import java.io.IOException;

import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;
import edu.ius.robotics.robots.x80pro.X80Pro;
import edu.ius.robotics.robots.x80pro.X80Pro.SensorDataType;

public class X80ProSonarTest implements IRobotEventHandler
{
	X80Pro robot;
	
	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		X80ProSonarTest test = new X80ProSonarTest();
		X80Pro robot = null;
		try
		{
			robot = new X80Pro("192.168.0.204", test);
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
		
		if (robot == null)
		{
			System.err.println("Unresolved error: No robot instance");
			return;
		}
	}
	
	@Override
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType)
	{
		if (sender == robot && SensorDataType.STANDARD == sensorDataType)
		{
			System.out.println(robot.getSonarRange(1));
		}
	}
	
	@Override
	public void audioDataReceivedEvent(IRobot sender, short[] audioData)
	{
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void imageDataReceivedEvent(IRobot sender, byte[] imageData)
	{
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void shutdownEvent(IRobot sender)
	{
		if (sender == robot)
		{
			robot.lowerHead();
		}
	}
	
	@Override
	public void startupEvent(IRobot sender)
	{
		this.robot = (X80Pro)sender;
		robot.raiseHead();
	}
}
