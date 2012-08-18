package edu.ius.robotics;

import edu.ius.robotics.X80Pro.X80Pro;
import java.lang.Math;

public class APLite implements Runnable
{
	public static final int ACCEPTABLE_ENCODER_DEVIATION = 16;
	public static final int NO_CONTROL = -32768;
	public static final int MODE_PWM = 0;
	public static final int MODE_POSITION = 1;
	public static final int MODE_VELOCITY = 2;

	public static final int INITIAL_TARGET_FINAL = 3;
	public static final int FINAL = 2;
	public static final int TARGET = 1;
	public static final int INITIAL = 0;
	public static final int TURN_QUAD_T = 4;

	public static final int MODE_SINGLEPOT = 0;
	public static final int MODE_DUALPOT = 1;
	public static final int MODE_QUADRATURE = 2;

	public static final int cFULL_COUNT = 32767;
	public static final int cWHOLE_RANGE = 1200;

	public static final int DIRECTION = -1;
	public static final int SERVO0_INI = 3650;
	public static final int SERVO1_INI = 3300;
	public static final int MAX_VELOCITY = 1200;
	public static final int VELOCITY = 1000;

	public static final int L_MAX_PWM = 32767;
	public static final int R_MAX_PWM = 0;
	public static final int N_PWM = 16383;
	public static final int O_PWM = 8000;
	public static final int DUTY_CYCLE_UNIT = 8383;

	public static final int MICRO_DELAY = 50;
	public static final int MINI_DELAY = 100;
	public static final int DELAY = 500;
	public static final int BIG_DELAY = 1000;
	public static final int UBER_DELAY = 5000;

	public static final short WHEEL_ENCODER_2PI_X80PRO = 800;

	public static final short MAX_IR = 81;
	public static final short MIN_IR = 9;

	public static final int L = 0;
	public static final int R = 1;

	public static final int NUM_IR_SENSORS = 7;
	public static final int NUM_IR_SENSORS_FRONT = 4;
	public static final int MAX_SPEED = 32767;
	public static final int RANGE = 30;

	// C++ floating point fields must be initialized at runtime:
	double WHEEL_DISPLACEMENT;
	double WHEEL_RADIUS;
	double PI;
	
	private double velocity;
	private double heading;
	private double step;
	private boolean isFullStop;
	
	private class sensors 
	{
		public static Double[] thetaIr = new Double[4]; 
	}
	
	X80Pro robot;
	
	APLite(X80Pro robot)
	{
		this.robot = robot;
		//resetHead();
		velocity = 0.70; // 70%
		heading = 0.0; // radians
		step = 1.0;
		sensors.thetaIr[0] = 0.785;
		sensors.thetaIr[1] = 0.35;
		sensors.thetaIr[2] = -0.35;
		sensors.thetaIr[3] = -0.785;
		isFullStop = true;
	}
	
	public void resetHead()
	{
		robot.servoNonTimeCtrlAll(SERVO0_INI, SERVO1_INI, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
	}
	
	public void suspendMotors()
	{
		robot.disableDcMotor(L);
		robot.disableDcMotor(R);		
	}
	
	void turn(double theta, int seconds)
	{
		//@ post: Robot has turned an angle theta in radians

	    int diffEncoder;
		int leftCmd;
		int rightCmd;

		diffEncoder = (int)((WHEEL_ENCODER_2PI_X80PRO*WHEEL_DISPLACEMENT*theta)/(4*PI*WHEEL_RADIUS));
		leftCmd = robot.getEncoderPulse(0) - diffEncoder;
		
		if (leftCmd < 0) leftCmd = 32767 + leftCmd;
		else if (leftCmd > 32767) leftCmd = leftCmd - 32767;
		rightCmd = robot.getEncoderPulse(1) - diffEncoder;
		
		if (rightCmd < 0) rightCmd = 32767 + rightCmd;
		else if (rightCmd > 32767) rightCmd = rightCmd - 32767;
		
		robot.setDcMotorPositionControlPid(L, 1000, 5, 10000);
		robot.setDcMotorPositionControlPid(R, 1000, 5, 10000);
		robot.dcMotorPositionNonTimeCtrlAll(leftCmd, rightCmd, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
	}
	
	public void run()
	{
		double v, w, F, kmid, kside, Fx, Fy, theta, FR, mass, alpha, heading, power[2];
		int i, x;

		step = mass = 1.0; // mass?  1 robot, maybe.
		kmid = 5.0; // hooke's law constant for two middle sensors
		kside = 4.0; // hooke's law constant two outside sensors
		FR = 1.0; // friction, nearly perfect?
		alpha = 1.0; // decides the amount to turn
		Fx = 100.0; // attractive goal force in x direction (forward)
		Fy = 0.0; // robot does not move sideways (without turning)
		v = velocity; // initial value is at 70%
		heading = heading; // current heading
		
		int count = 0;
		for (i = 0; i < NUM_IR_SENSORS_FRONT; ++i) 
		{
			if (robot.SensorIrRange(i) <= RANGE) { x = sensors.Ir[i]; ++count; } // object detected
			else x = 0; // nothing seen
			if (i == 1 || i == 2) F = kmid*x; // compute force from two inside sensors
			else F = kside*x; // compute force from two outside sensors
			Fx = Fx - (F*Math.cos(sensors.thetaIr[i])); // Repulsive x component
			Fy = Fy - (F*Math.sin(sensors.thetaIr[i])); // Repulsive y component
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
			power[L] = (int)(v - v*alpha*theta); // power to left motor (v - v*alpha*w)
			power[R] = (int)(v + v*alpha*theta); // power to right motor (v + v*alpha*w)
			power[L] /= 100*PI/2; power[R] /= 100*PI/2; // need precentages.
			if (power[L] > 1 || power[R] > 1) // if > 100% power output requested, cap the motor power.
				if (power[R] >= power[L]) { power[L] = power[L]/power[R]; power[R] = 1.0; }
				else { power[R] = power[R]/power[L]; power[L] = 1.0; }
			if (power[L] > 0) power[L] = N_PWM + O_PWM + (DUTY_CYCLE_UNIT/3)*power[L]; // 16384 + 8000 + (scaled) power[L]: faster!
			else if (power[L] < 0) power[L] = N_PWM - O_PWM + (DUTY_CYCLE_UNIT/3)*power[L]; // 16384 - 8000 + (scaled) power[L]: slower!
			else power[L] = N_PWM; // neutral PWM setting, 0% duty cycle
			//if (power[L] > L_MAX_PWM) power[L] = L_MAX_PWM; // L_MAX_PWM = 32767, +100% duty cycle
			//else if (power[L] < N_PWM) power[L] = N_PWM; // stand still, 0% duty cycle
			if (power[R] > 0) power[R] = N_PWM - O_PWM - (DUTY_CYCLE_UNIT/3)*power[R]; // reverse: 16384 - 8000 - power[R]
			else if (power[R] < 0) power[R] = N_PWM + O_PWM - (DUTY_CYCLE_UNIT/3)*power[R]; // reverse: 16384 + 8000 + power[R]
			else power[R] = N_PWM; // neutral PWM setting, 0% duty cycle
			//if (power[R] < R_MAX_PWM) power[R] = R_MAX_PWM; // yes, < .. negative threshold @ -100% duty cycle
			//else if (power[R] > N_PWM) power[R] = N_PWM; // stand still, 0% duty cycle
			robot.dcMotorPwmNonTimeCtrlAll((int)power[L], (int)power[R], NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
		//} else Robot.DcMotorPwmNonTimeCtrAll((short)N_PWM, (short)N_PWM, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL); Sleep(MICRO_DELAY);
	}
}
