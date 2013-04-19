package edu.ius.robotics.samples;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;
import edu.ius.robotics.robots.x80pro.X80Pro;

public class X80ProIRSensorTest implements IRobotEventHandler
{
	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		X80ProIRSensorTest test = new X80ProIRSensorTest();
		
		X80Pro robot = null;
		//X80Pro otherRobot = null;
		
		try
		{
			robot = new X80Pro("192.168.0.204", test);
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
		
		//robot.resumeAllSensors();
//		robot.resetHead();
		
//		while (true)
//		{
//			System.err.println("robot.getIRRange(0): " + robot.getIRRange(0));
//			System.err.println("robot.getIRRange(1): " + robot.getIRRange(1));
//			System.err.println("robot.getIRRange(2): " + robot.getIRRange(2));
//			System.err.println("robot.getIRRange(3): " + robot.getIRRange(3));
//			System.err.println();
//			try
//			{
//				Thread.sleep(100);
//			}
//			catch (InterruptedException e)
//			{
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//		}
		
		//robot.lowerHead();
		//robot.shutdown();
	}

	@Override
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType)
	{
		// TODO Auto-generated method stub
		System.err.println("robot.getIRRange(0): " + ((X80Pro) sender).getIRRange(0));
		System.err.println("robot.getIRRange(1): " + ((X80Pro) sender).getIRRange(1));
		System.err.println("robot.getIRRange(2): " + ((X80Pro) sender).getIRRange(2));
		System.err.println("robot.getIRRange(3): " + ((X80Pro) sender).getIRRange(3));
		System.err.println();
		
		try
		{
			Thread.sleep(100);
		}
		catch (InterruptedException ex)
		{
			ex.printStackTrace();
		}
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
