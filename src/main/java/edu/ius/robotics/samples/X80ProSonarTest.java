package edu.ius.robotics.samples;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;
import edu.ius.robotics.robots.x80pro.X80Pro;

public class X80ProSonarTest implements IRobotEventHandler
{
	
	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		X80Pro robot = null;
		//X80Pro otherRobot = null;
		
		try
		{
			robot = new X80Pro("192.168.0.204");
			//otherRobot = new X80Pro("192.168.0.202");
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
		
		if (robot == null) // || otherRobot == null)
		{
			return;
		}
		
		//otherRobot.resumeAllSensors();
		//otherRobot.resetHead();
		
		robot.resumeAllSensors();
		robot.resetHead();
		
		//robot.lowerHead();
		//robot.shutdown();
	}

	@Override
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void audioDataReceivedEvent(IRobot sender, short[] audioData)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void imageDataReceivedEvent(IRobot sender, ByteArrayOutputStream imageData)
	{
		// TODO Auto-generated method stub
		
	}
}
