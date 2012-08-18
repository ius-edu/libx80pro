package edu.ius.robotics.X80Pro;

import java.io.*;

import edu.ius.robotics.sockets.X80ProUdpSocket;
import edu.ius.robotics.boards.PMS5005;

/**
 * 
 * @author Bo Wei modifications by Jesse Riddle
 */
public class X80Pro implements IX80Pro, Runnable
{	
    /** minimum time step in milliseconds */
    public final int MIN_TIME_MILLI = 50;
	
    public final int nStdSensors = 6;
    public final int nMotors = 2;
    public final int nIrSensors = 8; 
    public final int nCustomSensors = 9;
    public final int nUltrasonicSensors = 6;
    public final int nHumanSensors = 2;
	
    // private ByteArrayInputStream byteArrIn;
    // private DataInputStream dataIn;
    // private DataOutputStream dataOut;
    // private InputStream inputStream;
    // private BufferedReader reader;
    // private String command;
    public int[] encoderPos;
    public double[] irDis;
    // motor sensor
    public int[] encoderSpeed;
    public double[] motorCurrent;
    // custom sensor data
    public int[] customAd;
    public int customIo;
    // standard sensor data
    public double[] usDis;
    public int[] humanAlarm;
    public int[] humanMotion;
    public int irRange;
    public double boardVol;
    public double dcMotorVol;
	
    private int[] motorSensorAry;
    private int[] standardSensorAry;
    private int[] customSensorAry;
	
    public final int HEADER_SIZE = 7;
    public final int PAYLOAD_OFFSET = 6;
    public final int DID_OFFSET = 4;
	
    /** wheel distance */
    final double WheelDis = 0.265;
    
    /** wheel radius */
    final double WheelR = 0.0825;
    
    /** encoder one circle count */
    final int CircleCnt = 1200;
	
    private X80ProUdpSocket socket;
    
    /**
     * Creates a new instance of X80Pro
     * @param robotIp
     * @param robotPort
     */
    public X80(String robotIp, int robotPort)
    public X80Pro(String robotIp, int robotPort)
    {
	this.makeSensorData();
	this.clearSensorData();
		
	if (!socket.connectRobot(robotIp, robotPort, MIN_TIME_MILLI))
	    System.exit(3);
    }
	
    /**
     * makeSensorData
     */
    private void makeSensorData()
    {
	this.encoderPos = new int[nMotors];
	this.irDis = new double[nIrSensors];
	this.encoderSpeed = new int[nMotors];
	this.motorCurrent = new double[nMotors];
	this.customAd = new int[nCustomSensors];
	this.usDis = new double[nUltrasonicSensors];
	this.humanAlarm = new int[nHumanSensors];
	this.humanMotion = new int[nHumanSensors];
    }
	
    /**
     * clearSensorData
     */
    public void clearSensorData()
    {
	int c, z;
		
	for (c = 0, z = this.encoderPos.length; c < z; ++c)
	{
	    this.encoderPos[c] = 0;
	}
		
	for (c = 0, z = this.irDis.length; c < z; ++c)
	{
	    this.irDis[c] = (double) (c + 1);
	}
		
	// motor sensor
	for (c = 0, z = this.encoderSpeed.length; c < z; ++c)
	{
	    this.encoderSpeed[c] = 0;
	}
		
	for (c = 0, z = this.motorCurrent.length; c < z; ++c)
	{
	    this.motorCurrent[c] = 0.0;
	}
		
	// custom sensor data
	for (c = 0, z = this.customAd.length; c < z; ++c)
	{
	    this.customAd[c] = 0;
	}
		
	this.customIo = 0;
		
	// standard sensor data
	for (c = 0, z = this.usDis.length; c < z; ++c)
	{
	    this.usDis[c] = 0;
	}
		
	for (c = 0, z = this.humanAlarm.length; c < z; ++c)
	{
	    this.humanAlarm[c] = 0;
	}
		
	for (c = 0, z = this.humanMotion.length; c < z; ++c)
	{
	    this.humanMotion[c] = 0;
	}
		
	this.irRange = 0;
	this.boardVol = 0;
	this.dcMotorVol = 0;
    }
	
    /**
     * receive packet thread
     */
    public void run()
    {
	socket.run();
    }
	
    /**
     * sends a command to the robot
     * @param command
     */
    public void sendCommand(byte[] command)
    {
	socket.send(command);
    }
	
    /**
     * analog to digital conversion
     * 
     * @param AdValue
     * @return
     */
    public static double Ad2Dis(int AdValue)
    {
	double distance = 0;
		
	if (AdValue > 4095 || AdValue <= 0)
	{
	    distance = 0;
	}
	else
	{
	    distance = 21.6 / ((double) (AdValue) * 3.0 / 4028 - 0.17);
			
	    if (80 < distance)
	    {
		distance = 0.81;
	    }
	    else if (distance < 10)
	    {
		distance = 0.09;
	    }
	    else
	    {
		distance = distance / 100.0;
	    }
	}
		
	return distance;
    }
    
    private void decodeSensorData(int[] sensorDataAry)
    {
	int z = sensorDataAry.length;
		
	if (sensorDataAry[0] == PMS5005.STX0 && sensorDataAry[1] == PMS5005.STX1 && 
	    sensorDataAry[z - 2] == PMS5005.ETX0 && sensorDataAry[z - 1] == PMS5005.ETX1)
	{
	    // here is a whole package (all sensor data)
	    // for first motor sensor data, please refer to the protocol
	    // documentation
	    if (sensorDataAry[DID_OFFSET] == PMS5005.GET_MOTOR_SENSOR_DATA)
	    {
		motorSensorAry = sensorDataAry;
		//encoderPos[0] = sensorDataAry[5 + 25] + (sensorDataAry[5 + 26]) * 256;
		//encoderSpeed[0] = sensorDataAry[5 + 27] + (sensorDataAry[5 + 28]) * 256;
		//encoderPos[1] = sensorDataAry[5 + 29] + (sensorDataAry[5 + 30]) * 256;
		//encoderSpeed[1] = sensorDataAry[5 + 31] + (sensorDataAry[5 + 32]) * 256;
				
		//motorCurrent[0] = (double) (sensorDataAry[5 + 13] + (sensorDataAry[5 + 14]) * 256) / 728.0;
		//motorCurrent[1] = (double) (sensorDataAry[5 + 15] + (sensorDataAry[5 + 16]) * 256) / 728.0;
	    }
	    else if (sensorDataAry[DID_OFFSET] == PMS5005.GET_CUSTOM_SENSOR_DATA)
	    {
		// custom sensor data
		customSensorAry = sensorDataAry;
		//for (int i = 0; i < 8; i++)
		//{
		//	customAd[i] = sensorDataAry[6 + 2 * i] + sensorDataAry[6 + 2 * i + 1] * 256;
		//}
				
		//customIo = sensorDataAry[5 + 17];
	    }
	    else if (sensorDataAry[DID_OFFSET] == PMS5005.GET_STANDARD_SENSOR_DATA)
	    {
		// standard sensors
		standardSensorAry = sensorDataAry;
		//for (int i = 0; i < nStdSensors; ++i)
		//{
		//	usDis[i] = (double) (sensorDataAry[nStdSensors + i]) / 100.0;
		//}
				
		//humanAlarm[0] = sensorDataAry[5 + 7] + sensorDataAry[5 + 8] * 256;
		//humanMotion[0] = sensorDataAry[5 + 9] + sensorDataAry[5 + 10] * 256;
		//humanAlarm[1] = sensorDataAry[5 + 11] + sensorDataAry[5 + 12] * 256;
		//humanMotion[1] = sensorDataAry[5 + 13] + sensorDataAry[5 + 14] * 256;
		//irRange = (sensorDataAry[5 + 25] + sensorDataAry[5 + 26] * 256);
		//boardVol = (double) (sensorDataAry[5 + 31] + sensorDataAry[5 + 32] * 256) / 4095.0 * 9.0;
		//dcMotorVol = (double) (sensorDataAry[5 + 33] + sensorDataAry[5 + 34] * 256) / 4095.0 * 24.0;
	    }
	}
		
	// the first ir sensor
	//irDis[0] = Ad2Dis(irRange);
		
	// the rest of the ir sensors
	//for (int i = 0; i < nIrSensors; i++)
	//{
	//	irDis[i + 1] = Ad2Dis(customAd[i + 1]);
	//}
    }
	
    public void motorSensorRequest(int packetNumber)
    {
	socket.send(PMS5005.motorSensorRequest((short)packetNumber));
    }

    public void standardSensorRequest(int packetNumber)
    {
	socket.send(PMS5005.standardSensorRequest((short)packetNumber));
    }

    public void customSensorRequest(int packetNumber)
    {
	socket.send(PMS5005.customSensorRequest((short)packetNumber));
    }

    public void allSensorRequest(int packetNumber)
    {
	socket.send(PMS5005.allSensorRequest((short)packetNumber));
    }

    public void enableMotorSensorSending()
    {
	socket.send(PMS5005.enableAllSensorSending());
    }

    public void enableStandardSensorSending()
    {
	socket.send(PMS5005.enableStandardSensorSending());
    }

    public void enableCustomSensorSending()
    {
	socket.send(PMS5005.enableCustomSensorSending());
    }

    public void enableAllSensorSending()
    {
	socket.send(PMS5005.enableAllSensorSending());
    }

    public void disableMotorSensorSending()
    {
	socket.send(PMS5005.disableMotorSensorSending());
    }

    public void disableStandardSensorSending()
    {
	socket.send(PMS5005.disableStandardSensorSending());
    }

    public void disableCustomSensorSending()
    {
	socket.send(PMS5005.disableCustomSensorSending());
    }

    public void disableAllSensorSending()
    {
	socket.send(PMS5005.disableAllSensorSending());
    }

    public void setMotorSensorPeriod(int timePeriod)
    {
	socket.send(PMS5005.setMotorSensorPeriod((short)timePeriod));
    }

    public void setStandardSensorPeriod(int timePeriod)
    {
	socket.send(PMS5005.setStandardSensorPeriod((short)timePeriod));
    }

    public void setCustomSensorPeriod(int timePeriod)
    {
	socket.send(PMS5005.setCustomSensorPeriod((short)timePeriod));
    }

    public void setAllSensorPeriod(int timePeriod)
    {
	socket.send(PMS5005.setAllSensorPeriod((short)timePeriod));
    }

    public int getSensorSonar(int channel)
    {
	return PMS5005.getSensorSonar((byte)channel, standardSensorAry);
    }

    public int getSensorIrRange(int channel)
    {
	return PMS5005.getSensorIrRange((byte)channel, standardSensorAry, customSensorAry);
    }

    public int getSensorHumanAlarm(int channel)
    {
	return PMS5005.getSensorHumanAlarm((byte)channel, standardSensorAry);
    }

    public int getSensorHumanMotion(int channel)
    {
	return PMS5005.getSensorHumanMotion((byte)channel, standardSensorAry);
    }

    public int getSensorTiltingX(int channel)
    {
	return PMS5005.getSensorTiltingX(standardSensorAry);
    }

    public int getSensorTiltingY(int channel)
    {
	return PMS5005.getSensorTiltingY(standardSensorAry);
    }

    public int getSensorOverheat(int channel)
    {
	return PMS5005.getSensorOverheat((byte)channel, standardSensorAry);
    }

    public int getSensorTemperature()
    {
	return PMS5005.getSensorTemperature(standardSensorAry);
    }

    public int getSensorIrCode(int index)
    {
	return PMS5005.getSensorIrCode((byte)index, standardSensorAry);
    }

    public void setInfraredControlOutput(int lowWord, int highWord)
    {
	PMS5005.setInfraredControlOutput((short)lowWord, (short)highWord);
    }

    public int getSensorBatteryAd(int channel)
    {
	return PMS5005.getSensorBatteryAd((short)channel, standardSensorAry);
    }

    public int getSensorRefVoltage()
    {
	return PMS5005.getSensorRefVoltage(standardSensorAry);
    }

    public int getSensorPotVoltage()
    {
	return PMS5005.getSensorPotVoltage(standardSensorAry);
    }

    public int getSensorPot(int channel)
    {
	return PMS5005.getSensorPot((byte)channel, motorSensorAry);
    }

    public int getMotorCurrent(int channel)
    {
	return PMS5005.getMotorCurrent((byte)channel, motorSensorAry);
    }

    public int getEncoderDirection(int channel)
    {
	return PMS5005.getEncoderDirection((byte)channel, motorSensorAry);
    }

    public int getEncoderPulse(int channel)
    {
	return PMS5005.getEncoderPulse((byte)channel, motorSensorAry);
    }

    public int getEncoderSpeed(int channel)
    {
	return PMS5005.getEncoderSpeed((byte)channel, motorSensorAry);
    }

    public int getCustomAd(int channel)
    {
	return PMS5005.getCustomAd((byte)channel, customSensorAry);
    }

    public int getCustomDIn(int channel)
    {
	return PMS5005.getCustomDIn((byte)channel, customSensorAry);
    }

    public void setCustomDOut(int ival)
    {
	socket.send(PMS5005.setCustomDOut((byte)ival));
    }

    public void setMotorPolarity(int channel, int polarity)
    {
	socket.send(PMS5005.setMotorPolarity((byte)channel, (byte)polarity));
    }

    @SuppressWarnings("deprecation")
    public void enableDcMotor(int channel)
    {
	socket.send(PMS5005.enableDcMotor((byte)channel));
    }

    @SuppressWarnings("deprecation")
    public void disableDcMotor(int channel)
    {
	socket.send(PMS5005.disableDcMotor((byte)channel));
    }

    public void resumeDcMotor(int channel)
    {
	socket.send(PMS5005.resumeDcMotor((byte)channel));
    }

    public void suspendDcMotor(int channel)
    {
	socket.send(PMS5005.suspendDcMotor((byte)channel));
    }

    public void setDcMotorPositionControlPid(int channel, int Kp, int Kd, int Ki_x100)
    {
	socket.send(PMS5005.setDcMotorPositionControlPid((byte)channel, (short)Kp, (short)Kd, (short)Ki_x100));
    }

    public void setDcMotorSensorFilter(int channel, int filterMethod)
    {
	socket.send(PMS5005.setDcMotorSensorFilter((byte)channel, (short)filterMethod));
    }

    public void setDcMotorSensorUsage(int channel, int sensorType)
    {
	socket.send(PMS5005.setDcMotorSensorUsage((byte)channel, (byte)sensorType));
    }

    public void setDcMotorControlMode(int channel, int controlMode)
    {
	socket.send(PMS5005.setDcMotorControlMode((byte)channel, (byte)controlMode));
    }

    public void dcMotorPositionTimeCtrl(int channel, int cmdValue, int timePeriod)
    {
	socket.send(PMS5005.dcMotorPositionTimeCtrl((byte)channel, (short)cmdValue, (short)timePeriod));
    }

    public void dcMotorPositionNonTimeCtrl(int channel, int cmdValue)
    {
	socket.send(PMS5005.dcMotorPositionNonTimeCtrl((byte)channel, (short)cmdValue));
    }

    public void dcMotorPwmTimeCtrl(int channel, int cmdValue, int timePeriod)
    {
	socket.send(PMS5005.dcMotorPwmTimeCtrl((byte)channel, (short)cmdValue, (short)timePeriod));
    }

    public void dcMotorPwmNonTimeCtrl(int channel, int cmdValue)
    {
	socket.send(PMS5005.dcMotorPwmNonTimeCtrl((byte)channel, (short)cmdValue));
    }

    public void dcMotorPositionTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
    {
	socket.send(PMS5005.dcMotorPositionTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6, (short)timePeriod));
    }

    public void dcMotorPositionNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
    {
	socket.send(PMS5005.dcMotorPositionNonTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6));
    }

    public void dcMotorVelocityTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
    {
	socket.send(PMS5005.dcMotorVelocityTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6, (short)timePeriod));
    }

    public void dcMotorVelocityNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
    {
	socket.send(PMS5005.dcMotorVelocityNonTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6));
    }

    public void dcMotorPwmTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
    {
	socket.send(PMS5005.dcMotorPwmTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6, (short)timePeriod));
    }

    public void dcMotorPwmNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
    {
	socket.send(PMS5005.dcMotorPwmNonTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6));
    }

    public void enableServo(int channel)
    {
	socket.send(PMS5005.enableServo((byte)channel));
    }

    public void disableServo(int channel)
    {
	socket.send(PMS5005.disableServo((byte)channel));
    }

    public void servoTimeCtrl(int channel, int cmdValue, int timePeriod)
    {
	socket.send(PMS5005.servoTimeCtrl((byte)channel, (short)cmdValue, (short)timePeriod));
    }

    public void servoNonTimeCtrl(int channel, int cmdValue)
    {
	socket.send(PMS5005.servoNonTimeCtrl((byte)channel, (short)cmdValue));
    }

    public void servoTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, int timePeriod)
    {
	socket.send(PMS5005.servoTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6, (short)timePeriod));
    }

    public void servoNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6)
    {
	socket.send(PMS5005.servoNonTimeCtrlAll((short)pos1, (short)pos2, (short)pos3, (short)pos4, (short)pos5, (short)pos6));
    }

    public void lcdDisplayPMS(String bmpFileName)
    {
	socket.send(PMS5005.lcdDisplayPMS(bmpFileName));
    }

    public void setDcMotorVelocityControlPID(byte channel, int Kp, int Kd, int Ki)
    {
	socket.send(PMS5005.setDcMotorVelocityControlPID(channel, Kp, Kd, Ki));
    }
}