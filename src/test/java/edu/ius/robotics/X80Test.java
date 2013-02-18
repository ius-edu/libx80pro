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
			robot = new X80Pro("192.168.0.204");
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
			return;
		}
		
		robot.resumeAllSensors();
		robot.resetHead();
		
		//robot.setBothDCMotorControlModes(X80Pro.PWM_CONTROL_MODE, X80Pro.PWM_CONTROL_MODE);
		robot.setBothDCMotorSensorUsages(X80Pro.SENSOR_USAGE_ENCODER, X80Pro.SENSOR_USAGE_ENCODER);
		robot.setBothDCMotorControlModes(X80Pro.CONTROL_MODE_VELOCITY, X80Pro.CONTROL_MODE_VELOCITY);
		robot.resumeBothDCMotors();
		
		//robot.setBothDCMotorPulsePercentages(100, 100);
		//robot.setBothDCMotorVelocities(3000, 3000);
		//robot.setBothDCMotorPulses(X80Pro.MAX_PWM_L, X80Pro.MAX_PWM_R);
		
		robot.setBothDCMotorVelocities(30000, 30000);
		
		try
		{
			Thread.sleep(3000);
		}
		catch (Exception ex)
		{
			ex.printStackTrace();
		}
		
		//robot.releaseHead();
		
//		while (true)
//		{
//			//byte[] customSensorData = robot.getStandardSensorData();
//			//for (int i = 0; i < customSensorData.length; ++i)
//			//{
//			//	System.err.printf("%d ", customSensorData[i]);
//			//}
//			//System.err.println();
//			System.err.println(robot.getSensorIRRange(0));
//			try
//			{
//				Thread.sleep(100);
//			}
//			catch (Exception ex)
//			{
//				ex.printStackTrace();
//			}
		
//			try
//			{
//				Runtime.getRuntime().exec("clear");
//			}
//			catch (Exception ex)
//			{
//				ex.printStackTrace();
//			}
//		}
				
//		System.err.println("Move robot");
//		robot.setBothDCMotorVelocities(3000, 3000);
//		
//		System.err.println("Hold the move for 3 seconds (Thread.sleep call)");
//		try
//		{
//			Thread.sleep(3000);
//		}
//		catch (InterruptedException e)
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
//		APLite aplite = new APLite(robot);
//		aplite.run();
//		
//		try
//		{
//			Thread.sleep(10000);
//		}
//		catch (InterruptedException e)
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
		robot.shutdown();
	}
}
