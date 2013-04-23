package edu.ius.robotics.robots.x80pro.data;

import java.nio.ByteBuffer;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.x80pro.X80Pro.SensorCount;
import static edu.ius.robotics.convenience.Type.*;

public class StandardSensorData
{
	volatile public int[] sonarDistance;
	volatile public int[] humanAlarm;
	volatile public int[] motionDetect;
	volatile public int[] tiltingAD;
	volatile public int[] overheatAD;
	volatile public int temperatureAD;
	volatile public int infraredRangeAD;
	volatile public int[] infraredCommand;
	volatile public int mainboardBatteryVoltageAD_0to9V;
	volatile public int motorBatteryVoltageAD_0to24V;
	volatile public int servoBatteryVoltageAD_0to9V;
	volatile public int referenceVoltageAD_Vcc;
	volatile public int potentiometerVoltageAD_Vref;
	
	public StandardSensorData()
	{
		sonarDistance = new int[SensorCount.NUM_SONAR_SENSORS];
		humanAlarm = new int[SensorCount.NUM_HUMAN_SENSORS];
		motionDetect = new int[SensorCount.NUM_HUMAN_SENSORS];
		tiltingAD = new int[SensorCount.NUM_TILTING_SENSORS];
		overheatAD = new int[SensorCount.NUM_TEMPERATURE_SENSORS];
		temperatureAD = -1;
		infraredRangeAD = -1;
		infraredCommand = new int[SensorCount.NUM_INFRARED_RECEIVERS];
		mainboardBatteryVoltageAD_0to9V = -1;
		motorBatteryVoltageAD_0to24V = -1;
		servoBatteryVoltageAD_0to9V = -1;
		referenceVoltageAD_Vcc = -1;
		potentiometerVoltageAD_Vref = -1;
	}
	
	public void setStandardSensorData(byte[] standardSensorData)
	{
		ByteBuffer data = ByteBuffer.wrap(standardSensorData);
		data.order(PMS5005.BYTE_ORDER);
		
		int offset = 0, i;
		
		for (i = 0; i < SensorCount.NUM_SONAR_SENSORS; ++i)
		{
			sonarDistance[i] = unsigned(data.get(offset + i));
		}
		offset += SensorCount.NUM_SONAR_SENSORS; // NUM_SONAR_SENSORS*sizeOfByte;
		
		for (i = 0; i < SensorCount.NUM_HUMAN_SENSORS; ++i)
		{
			humanAlarm[i] = data.getShort(offset + ((i*sizeOfShort) << 1));
		}
		offset += sizeOfShort; // skip one short value
		
		for (i = 0; i < SensorCount.NUM_HUMAN_SENSORS; ++i)
		{
			motionDetect[i] = data.getShort(offset + ((i*sizeOfShort) << 1));
		}
		offset += 3*sizeOfShort; // skip 3 short values
		
		for (i = 0; i < SensorCount.NUM_TILTING_SENSORS; ++i)
		{
			tiltingAD[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_TILTING_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_OVERHEAT_SENSORS; ++i)
		{
			overheatAD[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_OVERHEAT_SENSORS*sizeOfShort;
		
		temperatureAD = data.getShort(offset + i*sizeOfShort);
		offset += sizeOfShort;
		
		infraredRangeAD = data.getShort(offset);
		offset += sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_INFRARED_RECEIVERS; ++i)
		{
			infraredCommand[i] = data.get(offset + i); 
		}
		offset += SensorCount.NUM_INFRARED_RECEIVERS; // Sensors.NUM_INFRARED_RECEIVERS*sizeOfByte;
		
		mainboardBatteryVoltageAD_0to9V = data.getShort(offset);
		offset += sizeOfShort;
		
		motorBatteryVoltageAD_0to24V = data.getShort(offset);
		offset += sizeOfShort;
		
		servoBatteryVoltageAD_0to9V = data.getShort(offset);
		offset += sizeOfShort;
		
		referenceVoltageAD_Vcc = data.getShort(offset);
		offset += sizeOfShort;
		
		potentiometerVoltageAD_Vref = data.getShort(offset);
	}
}
