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

		robot.resetHead();
		try
		{
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("resuming motor 0");
		robot.enableDcMotor(0);
		
		try
		{
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("resuming motor 1");
		
		robot.enableDcMotor(1);
		
		try
		{
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Move robot forward");
		robot.dcMotorVelocityNonTimeCtrlAll(3000, 3000, NO_CTRL, NO_CTRL, NO_CTRL, NO_CTRL);
		
		try
		{
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Move for 3 seconds");
		try
		{
			Thread.sleep(3000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Suspend motor 0");
		robot.suspendDcMotor(0);
		
		try
		{
			Thread.sleep(1000);
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.err.println("Suspend motor 1");
		robot.suspendDcMotor(1);
		
		//APLite aplite = new APLite(robot);
		
		//aplite.resetHead();
	}
}
