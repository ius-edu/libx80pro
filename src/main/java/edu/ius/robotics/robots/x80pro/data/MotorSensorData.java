package edu.ius.robotics.robots.x80pro.data;

import java.nio.ByteBuffer;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.x80pro.X80Pro.SensorCount;
import static edu.ius.robotics.dependencies.Type.*;

public class MotorSensorData
{
	volatile public int[] potentiometerAD;
	volatile public int[] motorCurrentAD;
	volatile public int[] encoderPulse;
	volatile public int[] encoderSpeed;
	volatile public int[] encoderDirection;
	
	public MotorSensorData()
	{
		potentiometerAD = new int[SensorCount.NUM_POTENTIOMETER_AD_SENSORS];
		motorCurrentAD = new int[SensorCount.NUM_MOTOR_CURRENT_AD_SENSORS];
		encoderPulse = new int[SensorCount.NUM_ENCODER_PULSE_SENSORS];
		encoderSpeed = new int[SensorCount.NUM_ENCODER_SPEED_SENSORS];
		encoderDirection = new int[SensorCount.NUM_ENCODER_DIRECTION_SENSORS];
	}
	
	public void setMotorSensorData(byte[] motorSensorData)
	{
		ByteBuffer data = ByteBuffer.wrap(motorSensorData);
		data.order(PMS5005.BYTE_ORDER);
		
		int offset = 0, i;
		
		for (i = 0; i < SensorCount.NUM_POTENTIOMETER_AD_SENSORS; ++i)
		{
			potentiometerAD[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_POTENTIOMETER_AD_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_MOTOR_CURRENT_AD_SENSORS; ++i)
		{
			motorCurrentAD[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_MOTOR_CURRENT_AD_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_ENCODER_PULSE_SENSORS; ++i)
		{
			encoderPulse[i] = data.getShort(offset + (i*sizeOfShort) << 1);
		}
		offset += sizeOfShort; // single sensor jump
		
		for (i = 0; i < SensorCount.NUM_ENCODER_SPEED_SENSORS; ++i)
		{
			encoderSpeed[i] = data.getShort(offset + ((i*sizeOfShort) << 1));
		}
		offset += 3*sizeOfShort; // triple sensor jump
		
		for (i = 0; i < SensorCount.NUM_ENCODER_DIRECTION_SENSORS; ++i)
		{
			encoderDirection[i] = data.get(offset) & (i + 1);
		}
	}
}
