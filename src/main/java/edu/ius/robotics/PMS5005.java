package edu.ius.robotics;

public class PMS5005
{

	public byte crc(byte[] buf) {
		// TODO Auto-generated method stub
		return 0;
	}

	public void systemMotorSensorRequest(int packetNumber) {
		// TODO Auto-generated method stub
		
	}

	public void systemStandardSensorRequest(int packetNumber) {
		// TODO Auto-generated method stub
		
	}

	public void systemCustomSensorRequest(int packetNumber) {
		// TODO Auto-generated method stub
		
	}

	public void systemAllSensorRequest(int packetNumber) {
		// TODO Auto-generated method stub
		
	}

	public void enableMotorSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void enableStandardSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void enableCustomSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void enableAllSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void disableMotorSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void disableStandardSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void disableCustomSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void disableAllSensorSending() {
		// TODO Auto-generated method stub
		
	}

	public void setSysMotorSensorPeriod() {
		// TODO Auto-generated method stub
		
	}

	public void setSysStandardSensorPeriod() {
		// TODO Auto-generated method stub
		
	}

	public void setSysCustomSensorPeriod() {
		// TODO Auto-generated method stub
		
	}

	public void setSysAllSensorPeriod() {
		// TODO Auto-generated method stub
		
	}

	public int getSensorSonar(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorIrRange(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorHumanAlarm(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorHumanMotion(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorTiltingX(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorTiltingY(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorOverheat(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorTemperature() {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorIrCode(int index) {
		// TODO Auto-generated method stub
		return 0;
	}

	public void setInfraredControlOutput(int lowWord, int highWord) {
		// TODO Auto-generated method stub
		
	}

	public int getSensorBatteryAd(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorRefVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorPotVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getSensorPot(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getMotorCurrent(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderDir(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderPulse(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getEncoderSpeed(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getCustomAd(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public int getCustomDin(int channel) {
		// TODO Auto-generated method stub
		return 0;
	}

	public void setCustomDout(int ival) {
		// TODO Auto-generated method stub
		
	}

	public void setMotorPolarity(int channel, int polarity) {
		// TODO Auto-generated method stub
		
	}

	public void enableDcMotor(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void disableDcMotor(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void resumeDcMotor(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void suspendDcMotor(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorPositionControlPid(int channel, int Kp, int Kd,
			int Ki_x100) {
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorSensorFilter(int channel, int filterMethod) {
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorSensorUsage(int channel, int sensorType) {
		// TODO Auto-generated method stub
		
	}

	public void setDcMotorControlMode(int channel, int controlMode) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionTimeCtr(int channel, int cmdValue, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionNonTimeCtr(int channel, int cmdValue) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmTimeCtr(int channel, int cmdValue, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmNonTimeCtr(int channel, int cmdValue) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionTimeCtrAll(int pos1, int pos2, int pos3,
			int pos4, int pos5, int pos6, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPositionNonTimeCtrlAll(int pos1, int pos2, int pos3,
			int pos4, int pos5, int pos6) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorVelocityTimeCtrlAll(int pos1, int pos2, int pos3,
			int pos4, int pos5, int pos6, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorVelocityNonTimeCtrAll(int pos1, int pos2, int pos3,
			int pos4, int pos5, int pos6) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmTimeCtrAll(int pos1, int pos2, int pos3, int pos4,
			int pos5, int pos6, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void dcMotorPwmNonTimeCtrAll(int pos1, int pos2, int pos3, int pos4,
			int pos5, int pos6) {
		// TODO Auto-generated method stub
		
	}

	public void enableServo(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void disableServo(int channel) {
		// TODO Auto-generated method stub
		
	}

	public void servoTimeCtr(int channel, int cmdValue, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void servoNonTimeCtr(int channel, int cmdValue) {
		// TODO Auto-generated method stub
		
	}

	public void servoTimeCtrAll(int pos1, int pos2, int pos3, int pos4,
			int pos5, int pos6, int timePeriod) {
		// TODO Auto-generated method stub
		
	}

	public void servoNonTimeCtrAll(int pos1, int pos2, int pos3, int pos4,
			int pos5, int pos6) {
		// TODO Auto-generated method stub
		
	}

	public void lcdDisplayPMS(String bmpFileName) {
		// TODO Auto-generated method stub
		
	}

}
