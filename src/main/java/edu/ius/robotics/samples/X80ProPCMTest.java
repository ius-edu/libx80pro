package edu.ius.robotics.samples;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import edu.ius.robotics.robots.X80Pro;
import edu.ius.robotics.robots.interfaces.*;

public class X80ProPCMTest implements IRobotEventHandler
{
	private X80Pro robot;
	
	public X80Pro getRobot()
	{
		return robot;
	}

	public static void main(String[] args)
	{
		X80Pro robot = null;
		X80ProPCMTest pcmtest = new X80ProPCMTest();
		try
		{
			robot = new X80Pro("192.168.0.202", pcmtest, false);
		}
		catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (null == robot)
		{
			System.err.println("No robot instance!");
			return;
		}
		robot.startAudioRecording(4);
		robot.suspendAllSensors();
		try
		{
			Thread.sleep(5000);
		}
		catch (Exception ex)
		{
			ex.printStackTrace();
		}
		robot.stopAudioRecording();
	}
	
	@Override
	public void audioDataReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream audioBuffer)
	{
		// TODO Auto-generated method stub
		System.out.println("*** AUDIO DATA RECEIVED ***");
		System.out.println("Size: " + audioBuffer.size());
	}

	@Override
	public void imageDataReceivedEvent(String robotIP, int robotPort, ByteArrayOutputStream imageBuffer)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void sensorDataReceivedEvent(String robotIP, int robotPort, int sensorDataType)
	{
		// TODO Auto-generated method stub
		
	}
}
