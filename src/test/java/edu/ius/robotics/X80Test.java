package edu.ius.robotics;

import edu.ius.robotics.X80Pro;

public class X80Test
{
	public static final int NO_CTRL = -32768;
	public static final int SERVO0_INI = 361000;
	public static final int SERVO1_INI = 3300;
	
	public static void main(String[] args)
	{
		X80Pro robot = new X80Pro("192.168.0.203");
		
		System.err.println("Reset Head");
		robot.resetHead();
		
		System.err.println("Resume Both DC Motors");
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
		
		System.err.println("Suspend Both DC Motors");
		robot.suspendBothDCMotors();
		
		//APLite aplite = new APLite(robot);
		
		//aplite.resetHead();
	}
}
