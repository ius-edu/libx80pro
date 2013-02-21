package edu.ius.robotics;

import edu.ius.robotics.robots.X80Pro;

import java.lang.Math;

public class APLite implements Runnable
{
	public static final int MID_LEFT = 1;
	public static final int MID_RIGHT = 2;
	
	private double velocity;
//	private double heading;
//	private double step;
//	private boolean isFullStop;
	
	X80Pro robot;
	
	APLite(X80Pro robot)
	{
		this.robot = robot;
		//resetHead();
		velocity = 1.00; // 100% // = 0.7 // 70%
//		heading = 0.0; // radians
//		step = 1.0;
//		isFullStop = true;
	}
	
	public void directRobot()
	{
		System.err.println("APLite.directRobot(): begin");
		double v, F, kmid, kside, Fx, Fy, theta, alpha; //, w, friction, mass, heading;
		double[] power = new double[2];
		int i, x;
		
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
			int irr = (int)(100*robot.getSensorIRRange(i)); // range in CM
			System.err.println("robot.getSensorIRRange(" + i + "): " + irr);
			if (irr <= X80Pro.RANGE)
			{
				//x = Sensors.thetaIR[i];
				x = (int)(100*robot.getSensorIRRange(i));
				System.err.println("too close, or wide open?");
				System.err.println("in if statement: robot.getSensorIRRange(" + i + "): " + x); // range in CM
				//++count;
			} // object detected
			else
			{
				x = 0; // nothing seen
			}
			
			if (i == MID_LEFT || i == MID_RIGHT) // interior IR sensors
			{
				F = kmid*x; // compute force from two inside sensors
			}
			else // exterior IR sensors
			{
				F = kside*x; // compute force from two outside sensors
			}
			
			Fx = Fx - (F*Math.cos(X80Pro.Sensors.thetaIR[i])); // Repulsive x component
			Fy = Fy - (F*Math.sin(X80Pro.Sensors.thetaIR[i])); // Repulsive y component
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
		//if ((-PI/2.0 <= theta) && (theta <= PI/2.0)) 
		//{ // if theta is in 1st or 4th quadrant
		power[X80Pro.L] = (int)(v - v*alpha*theta); // power to left motor (v - v*alpha*w)
		power[X80Pro.R] = (int)(v + v*alpha*theta); // power to right motor (v + v*alpha*w)
			
		power[X80Pro.L] /= Math.PI/2; // get percentage output.
		power[X80Pro.R] /= Math.PI/2; // get percentage output.
		//}
		
		robot.setBothDCMotorPulsePercentages((int) power[X80Pro.L], (int) power[X80Pro.R]);
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
