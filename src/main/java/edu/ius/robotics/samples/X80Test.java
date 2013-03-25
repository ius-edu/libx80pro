package edu.ius.robotics.samples;

import java.io.IOException;

import edu.ius.robotics.robots.X80Pro;

public class X80Test
{
	public static void main(String[] args)
	{
		X80Pro robot = null;
		//X80Pro otherRobot = null;
		
		try
		{
			robot = new X80Pro("192.168.0.201");
			//otherRobot = new X80Pro("192.168.0.202");
		}
		catch (IOException ex)
		{
			ex.printStackTrace();
		}
		
		if (robot == null) // || otherRobot == null)
		{
			return;
		}
		
		//otherRobot.resumeAllSensors();
		//otherRobot.resetHead();
		
		robot.resumeAllSensors();
		robot.resetHead();
		
		robot.setBothDCMotorSensorUsages(X80Pro.SENSOR_USAGE_ENCODER);
		robot.setBothDCMotorControlModes(X80Pro.CONTROL_MODE_VELOCITY);
		
		robot.setBothDCMotorVelocities(0, 0);
		robot.resumeBothDCMotors();
		
		//robot.setBothDCMotorPulsePercentages(100, 100);
		//robot.setBothDCMotorPulses(X80Pro.MAX_PWM_L, X80Pro.MAX_PWM_R);
		
		//robot.setBothDCMotorVelocities(300, 300);
		
//		try
//		{
//			Thread.sleep(2600);
//		}
//		catch (Exception ex)
//		{
//			ex.printStackTrace();
//		}
		
		//robot.releaseHead();
		
//		while (true)
//		{
//			//byte[] standardSensorData = robot.getStandardSensorData();
//			//for (int i = 0; i < standardSensorData.length; ++i)
//			//{
//			//	System.err.printf("%d ", standardSensorData[i]);
//			//}
//			//System.err.println();
//			System.err.println("IR Range: " + robot.getSensorIRRange(0));
//			try
//			{
//				Thread.sleep(100);
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
		
//		int leftCmd[] = new int[3];
//		//int turn_t;
//		
//		leftCmd[0] = robot.getEncoderPulse(0);
//		double theta = Math.PI/2;
//		//turn_t = (int)(4*(theta/Math.PI/2));
//		leftCmd[1] = robot.turnThetaRadians(theta);
//		int encoderValue = robot.getEncoderPulse(0);
//		if (encoderValue <= leftCmd[1])
//		{
//			while (robot.getEncoderPulse(0) < leftCmd[1] - 3)
//			{
//				try
//				{
//					Thread.sleep(50);
//				}
//				catch (InterruptedException e)
//				{
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			}
//		}
//		else
//		{
//			while (leftCmd[1] < robot.getEncoderPulse(0) - 3)
//			{
//				try
//				{
//					Thread.sleep(50);
//				}
//				catch (InterruptedException e)
//				{
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			}
//		}
//		leftCmd[2] = robot.getEncoderPulse(0);
//		robot.suspendBothDCMotors();
//		theta = (4*Math.PI*X80Pro.WHEEL_RADIUS*(leftCmd[2] - leftCmd[0]))/(X80Pro.WHEEL_ENCODER_2PI*X80Pro.WHEEL_DISPLACEMENT);
//		if (leftCmd[0] < leftCmd[2])
//		{
//			theta *= -1;
//		}
		
		robot.turn(Math.PI/4, 4);
		
		try
		{
			Thread.sleep(20000);
		}
		catch (Exception ex)
		{
			ex.printStackTrace();
		}
		
		robot.lowerHead();
		robot.shutdown();
	}
}
