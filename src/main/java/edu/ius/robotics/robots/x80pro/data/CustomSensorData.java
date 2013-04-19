package edu.ius.robotics.robots.x80pro.data;

import java.nio.ByteBuffer;

import edu.ius.robotics.robots.boards.PMS5005;
import edu.ius.robotics.robots.x80pro.X80Pro.SensorCount;
import static edu.ius.robotics.dependencies.Type.*;

public class CustomSensorData
{
	volatile public int[] customAD;
	volatile public int[] ioPort;
	volatile public int[] mmDistanceToLeftConstellation;
	volatile public int[] mmDistanceToRightConstellation;
	volatile public int[] mmDistanceToRelativeConstellation;
	
	public CustomSensorData()
	{
		customAD = new int[SensorCount.NUM_CUSTOM_AD_SENSORS];
		ioPort = new int[SensorCount.NUM_IO_PORT_SENSORS];
		mmDistanceToLeftConstellation = new int[SensorCount.NUM_LEFT_CONSTELLATION_SENSORS];
		mmDistanceToRightConstellation = new int[SensorCount.NUM_RIGHT_CONSTELLATION_SENSORS];
		mmDistanceToRelativeConstellation = new int[SensorCount.NUM_RELATIVE_CONSTELLATION_SENSORS];
	}
	
	public void setCustomSensorData(byte[] customSensorData)
	{
		ByteBuffer data = ByteBuffer.wrap(customSensorData);
		data.order(PMS5005.BYTE_ORDER);
		
		int offset = 0, i;
		
		for (i = 0; i < SensorCount.NUM_CUSTOM_AD_SENSORS; ++i)
		{
			customAD[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_CUSTOM_AD_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_IO_PORT_SENSORS; ++i)
		{
			ioPort[i] = data.get(offset) & (0x01 << i);
		}
		offset += SensorCount.NUM_IO_PORT_SENSORS >> 3; // (Sensors.NUM_IO_PORT_SENSORS >> 3)*sizeOfByte;
		
		for (i = 0; i < SensorCount.NUM_LEFT_CONSTELLATION_SENSORS; ++i)
		{
			mmDistanceToLeftConstellation[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_LEFT_CONSTELLATION_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_RIGHT_CONSTELLATION_SENSORS; ++i)
		{
			mmDistanceToRightConstellation[i] = data.getShort(offset + i*sizeOfShort);
		}
		offset += SensorCount.NUM_RIGHT_CONSTELLATION_SENSORS*sizeOfShort;
		
		for (i = 0; i < SensorCount.NUM_RELATIVE_CONSTELLATION_SENSORS; ++i)
		{
			mmDistanceToRelativeConstellation[i] = data.getShort(offset + i*sizeOfShort);
		}
	}
}
