package edu.ius.robotics.robots.interfaces;

public interface IRobotAudio
{
	public void audioEvent(String robotIP, int robotPort, short[] audioData);
}
