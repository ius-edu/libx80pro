package edu.ius.robotics.samples;

import java.awt.image.BufferedImage;
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
		X80ProPCMTest pcmtest = new X80ProPCMTest();
		pcmtest.getRobot().startAudioRecording((byte) (0xFF & 255));
		pcmtest.getRobot().stopAudioRecording();
	}
	
	public X80ProPCMTest()
	{
		try
		{
			robot = new X80Pro("192.168.0.201", this);
		}
		catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
	}
	
	@Override
	public void audioEvent(String robotIP, int robotPort, short[] audioData)
	{
		
		// Do something with raw pcm waveform?
	}

	@Override
	public void imageEvent(String robotIP, int robotPort, BufferedImage imageData)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void sensorEvent(String robotIP, int robotPort, byte[] sensorData, int dataLength)
	{
		// TODO Auto-generated method stub
		
	}
	
}
