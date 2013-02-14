package edu.ius.robotics;

import edu.ius.robotics.robots.X80Pro;

import java.lang.Math;

public class APLite implements Runnable
{	
	private double velocity;
//	private double heading;
//	private double step;
//	private boolean isFullStop;
	
	private static class Sensors 
	{
		public static Double[] thetaIR = new Double[4]; 
	}
	
	X80Pro robot;
	
	APLite(X80Pro robot)
	{
		this.robot = robot;
		//resetHead();
		velocity = 0.70; // 70%
//		heading = 0.0; // radians
//		step = 1.0;
//		isFullStop = true;
		Sensors.thetaIR[0] = 0.785;
		Sensors.thetaIR[1] = 0.35;
		Sensors.thetaIR[2] = -0.35;
		Sensors.thetaIR[3] = -0.785;
	}
	
	public void directRobot()
	{
		System.err.println("APLite.directRobot(): begin");
		double v, F, kmid, kside, Fx, Fy, theta, alpha; //, w, friction, mass, heading;
		double[] power = new double[2];
		int i; // , x;
		double x;
		
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
		for (i = 0; i < X80Pro.NUM_IR_SENSORS_FRONT; ++i) 
		{
			double irr = robot.getSensorIRRange(i);
			System.err.println("robot.getSensorIRRange: " + irr);
			if (irr <= X80Pro.RANGE) 
			{
				//x = Sensors.thetaIR[i];
				x = robot.getSensorIRRange(i);
				System.err.println("in if statement: robot.getSensorIRRange: " + irr);
				//++count;
			} // object detected
			else
			{
				x = 0; // nothing seen
			}
			
			if (i == 1 || i == 2)
			{
				F = kmid*x; // compute force from two inside sensors
			}
			else
			{
				F = kside*x; // compute force from two outside sensors
			}
			
			Fx = Fx - (F*Math.cos(Sensors.thetaIR[i])); // Repulsive x component
			Fy = Fy - (F*Math.sin(Sensors.thetaIR[i])); // Repulsive y component
		}
		
		//if(count > 0){
		//	Fx /= count;
		//	Fy /= count;
		//}
		v = Math.sqrt(Fx*Fx + Fy*Fy); // v = magnitude of force vector
		theta = Math.atan2(Fy,Fx); // robot moves in 1st or 4th quadrant
		//if ((PI/2.0 < theta) && (theta <= PI)) w = PI/4.0 - theta; // turn toward 2nd quadrant
		//else if ((-PI < theta) && (theta < -PI/2.0)) w = -PI/4.0 - theta; // turn toward 3rd quadrant
		//else w = theta; // turn toward 1st or 4th quadrant
		//w = theta;
		//if ((-PI/2.0 <= theta) && (theta <= PI/2.0)) { // if theta is in 1st or 4th quadrant
			power[X80Pro.L] = (int)(v - v*alpha*theta); // power to left motor (v - v*alpha*w)
			power[X80Pro.R] = (int)(v + v*alpha*theta); // power to right motor (v + v*alpha*w)
			
			power[X80Pro.L] /= 100*Math.PI/2; // get percentage output.
			power[X80Pro.R] /= 100*Math.PI/2; // get percentage output.
		
		robot.setBothDCMotorPulsePercentages(power[X80Pro.L], power[X80Pro.R]);
		System.err.println("APLite.directRobot(): end");
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
