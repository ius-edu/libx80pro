package edu.ius.robotics;

import edu.ius.robotics.robots.X80Pro;

public class X80Test
{
	public static void main(String[] args)
	{
		X80Pro robot = new X80Pro("192.168.0.203");
		
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
		
		robot.suspendBothDCMotors();
		
		APLite aplite = new APLite(robot);
		
		aplite.run();
		
		robot.shutdown();
	}
}
