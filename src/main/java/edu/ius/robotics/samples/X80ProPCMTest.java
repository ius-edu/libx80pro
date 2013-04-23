package edu.ius.robotics.samples;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

import edu.ius.robotics.robots.interfaces.*;
import edu.ius.robotics.robots.x80pro.X80Pro;

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
			robot = new X80Pro("192.168.0.204", pcmtest, false, true, true);
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
	public void imageDataReceivedEvent(IRobot sender, byte[] imageData)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void audioDataReceivedEvent(IRobot sender, short[] audioData)
	{
		System.out.println("*** AUDIO DATA RECEIVED ***");
		System.out.println("Buffer Length: " + audioData.length);
		System.out.print("Buffer Content: ");
		for (int i = 0, z = audioData.length; i < z; ++i) {
			System.out.printf("%2x ", audioData[i]);
		}
		System.out.println();
	}

}
