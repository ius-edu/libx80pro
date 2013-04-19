package edu.ius.robotics.samples;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import edu.ius.robotics.robots.x80pro.X80Pro;

public class X80ProLCDTest
{
	public static void main(String[] args)
	{
		X80Pro robot = null;
		try
		{
			robot = new X80Pro("192.168.0.204");
		}
		catch (Exception ex)
		{
			ex.printStackTrace();
		}
		if (null == robot)
		{
			System.err.println("No robot instance");
			return;
		}
		BufferedImage bufferedImage = null;
		try
		{
			bufferedImage = ImageIO.read(new File("/tmp/lcdimage.bmp"));
			System.out.println("Setting LCD image");
			robot.setLCDDisplayPMS(bufferedImage);
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
	}
}
