package edu.ius.robotics.samples;

import edu.ius.robotics.robots.x80pro.X80Pro;

public class X80ProResetHeadTest
{
	
	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		// TODO Auto-generated method stub
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
		robot.raiseHead();
		try
		{
			Thread.sleep(5000);
		}
		catch (InterruptedException ex)
		{
			ex.printStackTrace();
		}
		robot.lowerHead();
	}
	
}
