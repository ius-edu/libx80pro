package edu.ius.robotics.samples;

import edu.ius.robotics.robots.X80Pro;

import java.lang.Math;

public class APLite implements Runnable
{
	public static final double RANGE = 0.30;
	public static double velocity = 1.0;
	
	X80Pro robot;
	
	APLite(X80Pro robot)
	{
		this.robot = robot;
	}
	
	public void directRobot()
	{
		double v, F, kmid, kside, Fx, Fy, theta, alpha; //, w, friction, mass, heading;
		double[] power = new double[2];
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
		for (i = 0; i < X80Pro.Sensors.NUM_INFRARED_SENSORS_FRONT; ++i) 
		{
			int irr = (int) (100*robot.getIRRange(i));
			System.err.println("robot.getSensorIRRange(" + i + "): " + irr);
			if (irr <= RANGE) // object detected
			{
				x = (int) (100*robot.getIRRange(i));
				System.err.println("object too close, or object too far? sometimes gives same readings (incorrect deltaV?)");
			}
			else // nothing seen
			{
				x = 0;
			}
			
			if (i == X80Pro.Sensors.FRONT_MID_LEFT_INFRARED_SENSOR_INDEX || 
				i == X80Pro.Sensors.FRONT_MID_RIGHT_INFRARED_SENSOR_INDEX) // interior IR sensors
			{
				F = kmid*x; // compute force from two inside sensors
			}
			else // exterior IR sensors
			{
				F = kside*x; // compute force from two outside sensors
			}
			
			Fx = Fx - (F*Math.cos(X80Pro.Sensors.INFRARED_SENSOR_POSITION[i])); // Repulsive x component
			Fy = Fy - (F*Math.sin(X80Pro.Sensors.INFRARED_SENSOR_POSITION[i])); // Repulsive y component
		}
		v = Math.sqrt(Fx*Fx + Fy*Fy); // v = magnitude of force vector
		theta = Math.atan2(Fy,Fx); // robot moves in 1st or 4th quadrant
		
		power[X80Pro.L] = (int)(v - v*alpha*theta); // power to left motor (v - v*alpha*w)
		power[X80Pro.R] = (int)(v + v*alpha*theta); // power to right motor (v + v*alpha*w)
		
		power[X80Pro.L] /= Math.PI/2; // get percentage output.
		power[X80Pro.R] /= Math.PI/2; // get percentage output.
		
		robot.setBothDCMotorPulsePercentages((int) power[X80Pro.L], (int) power[X80Pro.R]);
	}
	
	public void run()
	{
		for (int i = 0; i < 100; ++i)
		{
			directRobot();
			try
			{
				Thread.sleep(60);
			}
			catch (Exception ex)
			{
				ex.printStackTrace();
			}
			
		}
	}
}
