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
	
	void turn(double theta, int seconds)
	{
		//@ post: Robot has turned an angle theta in radians

	    int diffEncoder = (int)((X80Pro.WHEEL_ENCODER_2PI*X80Pro.WHEEL_DISPLACEMENT*theta)/(4*Math.PI*X80Pro.WHEEL_RADIUS));

	    int leftPulseWidth = robot.getEncoderPulse(0) - diffEncoder;
		if (leftPulseWidth < 0)
		{
			leftPulseWidth = 32767 + leftPulseWidth;
		}
		else if (32767 < leftPulseWidth)
		{
			leftPulseWidth = leftPulseWidth - 32767;
		}
		
		int rightPulseWidth = robot.getEncoderPulse(1) - diffEncoder;
		if (rightPulseWidth < 0) 
		{
			rightPulseWidth = 32767 + rightPulseWidth;
		}
		else if (32767 < rightPulseWidth)
		{
			rightPulseWidth = rightPulseWidth - 32767;
		}
		
		robot.setDCMotorPositionControlPID(X80Pro.L, 1000, 5, 10000);
		robot.setDCMotorPositionControlPID(X80Pro.R, 1000, 5, 10000);
		
		robot.setBothDCMotorPulses(leftPulseWidth, rightPulseWidth);
	}
	
	public void setPulseWidths(double leftPulseWidth, double rightPulseWidth)
	{
		// if > 100% power output requested, cap the motor power.
		if (1 < leftPulseWidth || 1 < rightPulseWidth)
		{
			if (leftPulseWidth <= rightPulseWidth) 
			{ 
				leftPulseWidth = leftPulseWidth/rightPulseWidth; 
				rightPulseWidth = 1.0; 
			}
			else
			{
				rightPulseWidth = rightPulseWidth/leftPulseWidth; 
				leftPulseWidth = 1.0;
			}
		}
		
		if (0 < leftPulseWidth)
		{
			// 16384 + 8000 + (scaled) power[L]: increase left motor velocity.
			leftPulseWidth = X80Pro.N_PWM + X80Pro.O_PWM + (X80Pro.DUTY_CYCLE_UNIT/3)*leftPulseWidth;
		}
		else if (leftPulseWidth < 0)
		{
			// 16384 - 8000 + (scaled) power[L]: reduce left motor velocity.
			leftPulseWidth = X80Pro.N_PWM - X80Pro.O_PWM + (X80Pro.DUTY_CYCLE_UNIT/3)*leftPulseWidth; 
		}
		else
		{
			// neutral PWM setting, 0% duty cycle, let left motor sit idle
			leftPulseWidth = X80Pro.N_PWM;
		}
		
		//if (power[L] > L_MAX_PWM)
		//{
		// L_MAX_PWM = 32767, +100% duty cycle
		//	power[L] = L_MAX_PWM;
		//}
		//else if (power[L] < N_PWM)
		//{
		// stand still, 0% duty cycle
		//	power[L] = N_PWM; 
		//}
		
		if (0 < rightPulseWidth)
		{
			rightPulseWidth = X80Pro.N_PWM - X80Pro.O_PWM - (X80Pro.DUTY_CYCLE_UNIT/3)*rightPulseWidth; // reverse: 16384 - 8000 - power[R]
		}
		else if (rightPulseWidth < 0)
		{
			rightPulseWidth = X80Pro.N_PWM + X80Pro.O_PWM - (X80Pro.DUTY_CYCLE_UNIT/3)*rightPulseWidth; // reverse: 16384 + 8000 + rightPulseWidth
		}
		else
		{
			rightPulseWidth = X80Pro.N_PWM; // neutral PWM setting, 0% duty cycle
		}
		//if (power[R] < R_MAX_PWM) power[R] = R_MAX_PWM; // yes, < .. negative threshold @ -100% duty cycle
		//else if (power[R] > N_PWM) power[R] = N_PWM; // stand still, 0% duty cycle
		//robot.dcMotorPwmNonTimeCtrlAll((int)power[L], (int)power[R], NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
		robot.setBothDCMotorPulses((int)leftPulseWidth, (int)rightPulseWidth);
	//} else Robot.DcMotorPwmNonTimeCtrAll((short)N_PWM, (short)N_PWM, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL); Sleep(MICRO_DELAY);
	}
	
	public void run()
	{
		System.err.println("APLite.run(): begin");
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
			if (robot.getSensorIRRange(i) <= X80Pro.RANGE) 
			{
				//x = Sensors.thetaIR[i];
				x = robot.getSensorIRRange(i);
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
			
		setPulseWidths(power[X80Pro.L], power[X80Pro.R]);
		System.err.println("APLite.run(): end");
	}
}
