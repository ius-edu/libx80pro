package edu.ius.robotics.samples;

import edu.ius.robotics.robots.interfaces.IRobot;
import edu.ius.robotics.robots.interfaces.IRobotEventHandler;
import edu.ius.robotics.robots.x80pro.X80Pro;

import java.io.ByteArrayOutputStream;
import java.lang.Math;

public class APLite implements IRobotEventHandler
{
	public static final int PERSONAL_SPACE_IN_CM = 30;
	public static double velocity = 1.0;
	
	public static void main(String args[])
	{
		APLite aplite = new APLite();
		X80Pro robot = null;
		try
		{
			robot = new X80Pro("192.168.0.204", aplite);
		}
		catch (Exception ex)
		{
			ex.printStackTrace();
		}
		
		if (null == robot)
		{
			System.err.println("No robot instance!");
			return;
		}
	}
	
	synchronized public void direct(IRobot robot)
	{
		double v, F, kmid, kside, Fx, Fy, theta, alpha; //, w, friction, mass, heading;
		double powerL;
		double powerR;
		int i;
		int x;
		
		//step = mass = 1.0; // mass?  1 robot, maybe.
		//friction = 1.0; // friction, nearly perfect?
		kmid = 5.0; // hooke's law constant for two middle sensors
		kside = 4.0; // hooke's law constant two outside sensors
		alpha = 1.0; // decides the amount to turn
		Fx = 100.0; // attractive goal force in x direction (forward)
		Fy = 0.0; // robot does not move sideways (without turning)
		v = velocity; // initial value is at 70%
		//heading = heading; // current heading
		
		//int count = 0;
		for (i = 0; i < X80Pro.SensorCount.NUM_INFRARED_SENSORS_FRONT; ++i) 
		{
			int reading = ((X80Pro) robot).getIRRangeInCM(i);
			System.err.println("robot.getIRRange(" + i + "): " + reading);
			if (reading <= PERSONAL_SPACE_IN_CM) // object detected
			{
				if (X80Pro.SensorDistance.MIN_IR_DISTANCE_IN_CM != reading) x = reading;
				else x = 0;
			}
			else x = 0;
			
			if (X80Pro.Sensor.FRONT_LEFT_MID_IR_SENSOR == i || i == X80Pro.Sensor.FRONT_RIGHT_MID_IR_SENSOR) // interior IR sensors
			{
				F = kmid*x; // compute force from two inside sensors
			}
			else // exterior IR sensors
			{
				F = kside*x; // compute force from two outside sensors
			}
			
			Fx = Fx - (F*Math.cos(X80Pro.SensorDirection.IR_SENSOR_DIRECTION[i])); // Repulsive x component
			Fy = Fy - (F*Math.sin(X80Pro.SensorDirection.IR_SENSOR_DIRECTION[i])); // Repulsive y component
		}
		
		v = Math.sqrt(Fx*Fx + Fy*Fy); // v = magnitude of force vector
		theta = Math.atan2(Fy, Fx); // robot moves in 1st or 4th quadrant
		
		powerL = (int)(v - v*alpha*theta); // power to left motor (v - v*alpha*w)
		powerR = (int)(v + v*alpha*theta); // power to right motor (v + v*alpha*w)
		
		//powerL /= Math.PI/2; // get percentage output.
		//powerR /= Math.PI/2; // get percentage output.
		
		System.err.println();
		System.err.println("powerL: " + powerL);
		System.err.println("powerR: " + powerR);
		System.err.println();
		
		((X80Pro) robot).setBothDCMotorPulsePercentages((int) powerL, (int) powerR);
	}
	
	@Override
	public void sensorDataReceivedEvent(IRobot sender, int sensorDataType)
	{
		// TODO Auto-generated method stub
		if (X80Pro.SensorDataType.TYPE_STANDARD == sensorDataType || sensorDataType == X80Pro.SensorDataType.TYPE_CUSTOM)
		{
			direct(sender);
		}
	}
	
	@Override
	public void audioDataReceivedEvent(IRobot sender, short[] audioData)
	{
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void imageDataReceivedEvent(IRobot sender, ByteArrayOutputStream imageData)
	{
		// TODO Auto-generated method stub
		
	}
}
