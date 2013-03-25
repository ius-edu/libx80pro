package edu.ius.robotics.robots.interfaces;

import java.awt.image.BufferedImage;

public interface IRobotEventHandler
{
	public void audioEvent(String robotIP, int robotPort, short[] audioData);
	public void imageEvent(String robotIP, int robotPort, BufferedImage imageData);
	public void sensorEvent(String robotIP, int robotPort, byte[] sensorData, int dataLength);
}
