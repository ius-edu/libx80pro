package edu.ius.robotics.samples;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import edu.ius.robotics.robots.X80Pro;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;

public class X80ProImageTest implements IRobotEventHandler
{
	public static void main(String[] args) throws IOException
	{
		X80ProImageTest imageTest = new X80ProImageTest();
		X80Pro robot = new X80Pro("192.168.0.204", imageTest);
		robot.resetHead();
		
		try
		{
			System.out.println("Sleeping for 1 sec");
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println("Suspending all sensors");
		robot.suspendAllSensors();
		System.out.println("All sensors suspended");
		
		try
		{
			System.out.println("Sleeping for 1 sec");
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Take a photo");
		robot.takePhoto();
		
		try
		{
			System.err.println("Sleeping for 10 sec");
			Thread.sleep(10000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		robot.lowerHead();
	}
	
	@Override
	public void imageEvent(String robotIP, int robotPort, BufferedImage bufferedImage)
	{
		System.err.println("imageEvent raised");
        File imageFile = new File("/tmp/outputImage.bmp");
        try
		{
			ImageIO.write(bufferedImage, "bmp", imageFile);
			System.err.println("Wrote bmp (outputImage.bmp) to disk");
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
		// TODO Auto-generated method stub
		
	}

	@Override
	public void sensorEvent(String robotIP, int robotPort, byte[] sensorData, int dataLength)
	{
		// TODO Auto-generated method stub
		
	}
}
