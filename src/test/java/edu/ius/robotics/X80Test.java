package edu.ius.robotics;

import java.io.IOException;

import edu.ius.robotics.robots.X80Pro;

public class X80Test
{
	public static void main(String[] args)
	{
		X80Pro robot = null;
		
		try
		{
			robot = new X80Pro("192.168.0.203");
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			return;
		}
		
		robot.resumeAllSensors();
		robot.resetHead();
		robot.resumeBothDCMotors();
		
		System.err.println("Move robot");
		robot.setBothDCMotorVelocities(3000, 3000);
		
		System.err.println("Hold the move for 3 seconds (Thread.sleep call)");
		try
		{
			Thread.sleep(3000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		APLite aplite = new APLite(robot);
		aplite.run();
		
		try
		{
			// Let APLite run for 15s
			Thread.sleep(15000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		robot.suspendBothDCMotors();
		robot.shutdown();
	}
}
