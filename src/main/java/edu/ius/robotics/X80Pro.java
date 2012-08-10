
package edu.ius.robotics;

import com.drrobot.*;

public class X80Pro implements IX80{

    /* The structure for a packet to the PMS5005 is the following:
     * STX0 = 94 (always)
     * STX1 = 2 (always)
     * RID = 1 (generally)
     * Reserved = 0 (generally)
     * DID (Data ID) = <DID> (your choice of DID)
     * LENGTH = <len> (Length from next element to crc byte)
     * DATA = <data> (may be more than 1 byte in length)
     * CHECKSUM = <cksum> (use crc() method to calculate on cmd)
     * ETX0 = 94 (always)
     * ETX1 = 13 (always)
     */

    /* Start transmission, End transmission */
    public static final byte STX0 = 94;
    public static final byte STX1 = 2;
    public static final byte ETX0 = 94;
    public static final byte ETX1 = 13;
    
    /* Data ID (DID) descriptor listing */
    public static final byte DID_POSITIONCTRL = 3;
    public static final byte DID_POSITIONCTRLALL = 4;
    public static final byte DID_PWMCTRL = 5;
    public static final byte DID_PWMCTRLALL = 6;
    public static final byte DID_PARAMSET = 7;
    public static final byte DID_POWERCTRL = 22;
    public static final byte DID_LCDCTRL = 23;
    public static final byte DID_VELOCITYCTRL = 26;
    public static final byte DID_VELOCITYCTRLALL = 27;
    public static final byte DID_SERVOCTRL = 28;
    public static final byte DID_SERVOCTRLALL = 29;
    public static final byte DID_MOTORCTRL = 30;
    public static final byte DID_CONSTELLATIONCTRL = 80;
    public static final byte DID_GETMOTORSIGNAL = 123;
    public static final byte DID_GETCUSTOMSIGNAL = 124;
    public static final byte DID_GETSENSORDATA = 125;
    public static final byte DID_GETSENSORDATAALL = 127;
    // to use as ubyte: (byte)(DID_SETUPCOM & 0xff)
    public static final int DID_SETUPCOM = 255;
    /* End Data ID (DID) descriptor listing */

    public static final byte DCMOTORCTRLMODE = 14;
    
    public static final byte DCPOSITIONPID = 7;      // positon PID Control
    public static final byte DCVELOCITYPID = 8;      // velocity PID Control
    
    public static final byte PWMCTRL = 0;
    public static final byte POSITIONCTRL = 1;
    public static final byte VELOCITYCTRL = 2;
    
    public static final byte KpID = 1; // progressive id
    public static final byte KdID = 2; // derivative id
    public static final byte KiID = 3; // integral id
    
    public static final int NONCTRLCMD = 0xffff;      // no ctrl command
    public static final int NO_CONTROL = 0x8000;

    RobotSocket socket;

    X80Pro(RobotSocket socket) 
    {
    	this.socket = socket;
    }

    final double WheelDis = 0.265;      //wheel distance
    final double WheelR = 0.0825;       //wheel radius
    final int CircleCnt = 1200;     //encoder one circle count
    
    boolean connectFlag;

    @Override public byte crc(byte[] buf) {
	byte shift_reg, sr_lsb, data_bit, v;
	byte fb_bit;
	int z;
	shift_reg = 0; // initialize the shift register
	z = buf.length - 5;
	for (int i = 0 ; i < z; ++i) {
	    v = (byte) (buf[2 + i]); // start from RID
	    // for each bit
	    for (int j = 0; j < 8; ++j) {
		// isolate least sign bit
		data_bit = (byte) ((v & 0x01) & 0xff);
		sr_lsb = (byte) ((shift_reg & 0x01) & 0xff);
		// calculate the feed back bit
		fb_bit = (byte) (((data_bit ^ sr_lsb) & 0x01) & 0xff);
		shift_reg = (byte) ((shift_reg & 0xff) >>> 1);
		if (fb_bit == 1)
		    shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xff);
		v = (byte) ((v & 0xff) >>> 1);
	    }
	}
	return shift_reg;
    }    


    @Override public void systemMotorSensorRequest(int packetNumber) {
	byte[] cmd = new byte[9];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_GETMOTORSIGNAL;
        cmd[5] = 2; // len
	cmd[6] = (byte)(packetNumber & 0xff);
	cmd[7] = crc(cmd);
	cmd[8] = ETX0;
	cmd[9] = ETX1;
	socket.sendCommand(cmd);
    }


    @Override public void systemStandardSensorRequest(int packetNumber) {
	byte[] cmd = new byte[9];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_GETMOTORSIGNAL;
	cmd[5] = 2; // len
	cmd[6] = (byte)(packetNumber & 0xff);
	cmd[7] = crc(cmd);
	cmd[8] = ETX0;
	cmd[9] = ETX1;
	socket.sendCommand(cmd);
    }


    @Override public void servoTimeCtrAll(int pos1, int pos2, int pos3, 
            int pos4, int pos5, int pos6, int timePeriod) {
	byte[] cmd = new byte[23];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_SERVOCTRLALL;
	cmd[5] = 14; // len
	cmd[6] = (byte)(pos1 & 0xff);
        cmd[7] = (byte)((pos1 >>> 8) & 0xff);
        cmd[8] = (byte)(pos2 & 0xff);
        cmd[9] = (byte)((pos2 >>> 8) & 0xff);
        cmd[10] = (byte)(pos3 & 0xff);
        cmd[11] = (byte)((pos3 >>> 8) & 0xff);
        cmd[12] = (byte)(pos4 & 0xff);
        cmd[13] = (byte)((pos4 >>> 8) & 0xff);
        cmd[14] = (byte)(pos5 & 0xff);
        cmd[15] = (byte)((pos5 >>> 8) & 0xff);
        cmd[16] = (byte)(pos6 & 0xff);
        cmd[17] = (byte)((pos6 >>> 8) & 0xff);
        cmd[18] = (byte)(timePeriod & 0xff);
        cmd[19] = (byte)((timePeriod >>> 8) & 0xff);
	cmd[20] = crc(cmd);
	cmd[21] = ETX0;
	cmd[22] = ETX1;
	socket.sendCommand(cmd);
    }

    
    @Override public void suspendDcMotor(int channel) {
	byte[] cmd = new byte[9];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_MOTORCTRL;
	cmd[5] = 2; // len
	cmd[6] = 0;
	cmd[7] = (byte)(channel & 0xff);
        cmd[8] = crc(cmd);
	cmd[9] = ETX0;
	cmd[10] = ETX1;
	socket.sendCommand(cmd);
    }


    @Override public void servoTimeCtr(int channel, int cmdValue, 
            int timePeriod) {
        byte[] cmd = new byte[15];
        cmd[0] = STX0;
	cmd[1] = STX1;
        cmd[2] = 1;
	cmd[3] = 0;
        cmd[4] = DID_SERVOCTRL;
        cmd[5] = 6;
        cmd[6] = (byte)(channel & 0xff);
        cmd[7] = (byte)(cmdValue & 0xff);              // lsb
        cmd[8] = (byte)((cmdValue >>> 8) & 0xff);      // msb
        cmd[9] = DID_SERVOCTRL;                        // flag
        cmd[10] = (byte)(timePeriod & 0xff);           // lsb
        cmd[11] = (byte)((timePeriod >>> 8) & 0xff);   // msb
        cmd[12] = crc(cmd);
        cmd[13] = ETX0; 
	cmd[14] = ETX1;
        socket.sendCommand(cmd);
    }


    @Override public void dcMotorVelocityNonTimeCtrAll(int pos1, int pos2, 
            int pos3, int pos4, int pos5, int pos6) {
        byte[] cmd = new byte[21];
        cmd[0] = STX0;
	cmd[1] = STX1;
        cmd[2] = 1;
	cmd[3] = 0;
        cmd[4] = DID_VELOCITYCTRLALL;
        cmd[5] = 12;
        cmd[6] = (byte)(pos1 & 0xff);             // motor 1
        cmd[7] = (byte)((pos1 >>> 8) & 0xff);             
        cmd[8] = (byte)(pos2 & 0xff);             // motor 2
        cmd[9] = (byte)((pos2 >>> 8) & 0xff);
        cmd[10] = (byte)(pos3 & 0xff);            // motor 3
        cmd[11] = (byte)((pos3 >>> 8) & 0xff);
        cmd[12] = (byte)(pos4 & 0xff);            // motor 4
        cmd[13] = (byte)((pos4 >>> 8) & 0xff);
        cmd[14] = (byte)(pos5 & 0xff);            // motor 5
        cmd[15] = (byte)((pos5 >>> 8) & 0xff);           
        cmd[16] = (byte)(pos6 & 0xff);            // motor 6
        cmd[17] = (byte)((pos6 >>> 8) & 0xff);            
        cmd[18] = crc(cmd);
        cmd[19] = ETX0; 
	cmd[20] = ETX1;
        socket.sendCommand(cmd);
    }
    
    
    @Override public void dcMotorPositionTimeCtrAll(int cmd1, int cmd2, 
            int cmd3, int cmd4, int cmd5, int cmd6, int time) {
	byte[] cmd = new byte[23];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_POSITIONCTRLALL;
	cmd[5] = 14;
        cmd[6] = (byte)(cmd1 & 0xff);             // channel 1
        cmd[7] = (byte)((cmd1 >>> 8) & 0xff);
        cmd[8] = (byte)(cmd2 & 0xff);             // channel 2
        cmd[9] = (byte)((cmd2 >>> 8) & 0xff);
        cmd[10] = (byte)(cmd3 & 0xff);            // channel 3
        cmd[11] = (byte)((cmd3 >>> 8) & 0xff);
        cmd[12] = (byte)(cmd4 & 0xff);            // channel 4
        cmd[13] = (byte)((cmd4 >>> 8) & 0xff);
        cmd[14] = (byte)(cmd5 & 0xff);            // channel 5
        cmd[15] = (byte)((cmd5 >>> 8) & 0xff);
        cmd[16] = (byte)(cmd6 & 0xff);            // channel 6
        cmd[17] = (byte)((cmd6 >>> 8) & 0xff);
	cmd[18] = (byte)(time & 0xff);
	cmd[19] = (byte)((time >>> 8) & 0x0ff);
	cmd[20] = crc(cmd);
	cmd[21] = ETX0; 
	cmd[22] = ETX1;
        socket.sendCommand(cmd);
    }

    
    @Override public void setDcMotorControlMode(int channel, int controlMode) {
	byte[] cmd = new byte[12];
	cmd[0] = STX0; 
	cmd[1] = STX1;
	cmd[2] = 1; 
	cmd[3] = 0;
	cmd[4] = DID_PARAMSET;
	cmd[5] = 3;
	cmd[6] = DCMOTORCTRLMODE;
	cmd[7] = (byte)(channel & 0xff);
	// 0 -- PWM control, 1 -- position control, 2 -- velocity control
	cmd[8] = (byte)(controlMode & 0xff);
	cmd[9] = crc(cmd);
	cmd[10] = ETX0; 
	cmd[11] = ETX1;
	socket.sendCommand(cmd);
    }

    
    @Override public void setDcMotorPositionControlPid(int channel, int Kp, 
            int Kd, int Ki_x100) {
	byte[] cmd = new byte[20];
	cmd[0] = STX0;
	cmd[1] = STX1;
	cmd[2] = 1;
	cmd[3] = 0;
	cmd[4] = DID_PARAMSET;
	cmd[5] = 11;
	cmd[6] = DCPOSITIONPID;
	cmd[7] = (byte)(channel & 0xff);
	cmd[8] = KpID;
        cmd[9] = (byte)(Kp & 0xff);
        cmd[10] = (byte)((Kp >>> 8) & 0xff);
	cmd[11] = KdID;
        cmd[12] = (byte)(Kd & 0xff);
        cmd[13] = (byte)((Kd >>> 8) & 0xff);
	cmd[14] = KiID;
        cmd[15] = (byte)(Ki_x100 & 0xff);
        cmd[16] = (byte)((Ki_x100 >>> 8) & 0xff);
	cmd[17] = crc(cmd);
	cmd[18] = ETX0; 
	cmd[19] = ETX1;
	socket.sendCommand(cmd);
    }

    /**
     * 
     */
    private void setDcMotorVelocityControlPid(byte channel, int Kp, int Kd, 
            int Ki_x100) {
	byte[] cmd = new byte[20];
	cmd[0] = STX0; 
	cmd[1] = STX1;
	cmd[2] = 1; 
	cmd[3] = 0;
	cmd[4] = DID_PARAMSET;
	cmd[5] = 11;
	cmd[6] = DCVELOCITYPID;
	cmd[7] = channel;
	cmd[8] = KpID;
        cmd[9] = (byte)(Kp & 0xff);
        cmd[10] = (byte)((Kp >>> 8) & 0xff);
	cmd[11] = KdID;
        cmd[12] = (byte)(Kd & 0xff);
        cmd[13] = (byte)((Kd >>> 8) & 0xff);
	cmd[14] = KiID;
        cmd[15] = (byte)(Ki_x100 & 0xff);
        cmd[16] = (byte)((Ki_x100 >>> 8) & 0xff);
	cmd[17] = crc(cmd);
	cmd[18] = ETX0; 
	cmd[19] = ETX1;
	socket.sendCommand(cmd);
    }

    private void runStep(double runDis){
        //the robot will go forward the rundistance
        int diffEncoder = (int)( (runDis / ( 2 * Math.PI * WheelR ) ) * CircleCnt);
        
        int LeftTarget = socket.EncoderPos[0] - diffEncoder;
        if(LeftTarget < 0){
            LeftTarget = 32767 + LeftTarget;
        }
        else if(LeftTarget > 32767 ){
            LeftTarget = LeftTarget - 32767;
        }
        
        int RightTarget = socket.EncoderPos[1] + diffEncoder;
        if(RightTarget > 32767){
            RightTarget = RightTarget - 32767;
        }
        else if(RightTarget < 0){
            RightTarget = 32767 + RightTarget;
        }
        
        
        setDcMotorControlMode(0, POSITIONCTRL);
        setDcMotorControlMode(1, POSITIONCTRL);
        setDcMotorPositionControlPid(0,1000,30,2000);
        setDcMotorPositionControlPid(1,1000,30,2000);
        
        dcMotorPositionTimeCtrAll(LeftTarget,RightTarget,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,10000);
  
    }
    
    private void turnStep(double angle){
        //the parameter is degree, need transfor to radian
        angle = angle / 180 * Math.PI;
        int diffEncoder = (int)(( ( angle * WheelDis / 2) / (2 * Math.PI * WheelR)) * CircleCnt );
        int LeftTarget = socket.EncoderPos[0] + diffEncoder;
        if(LeftTarget < 0){
            LeftTarget = 32767 + LeftTarget;
        }
        else if(LeftTarget > 32767 ){
            LeftTarget = LeftTarget - 32767;
        }
        
        int RightTarget = socket.EncoderPos[1] + diffEncoder;
        if(RightTarget > 32767){
            RightTarget = RightTarget - 32767;
        }
        else if(RightTarget < 0){
            RightTarget = 32767 + RightTarget;
        }
        
        setDcMotorControlMode( (byte)0, POSITIONCTRL);
        setDcMotorControlMode( (byte)1, POSITIONCTRL);
        
        dcMotorPositionTimeCtrAll(LeftTarget,RightTarget,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,5000);
    }
    
    public boolean isConnected() {
	return connectFlag;
    }

    public void connect(String ip, int port) {
	socket = new RobotSocket(ip, port);
	if (socket == null) connectFlag = false;
	else connectFlag = true;
        /*
	pms5005 = new PMS5005(socket);
	pmb5010 = new PMB5010(socket);
         */
    }

    public void disconnect() {
        /*
	pmb5010 = null;
	pms5005 = null;
         */
	socket = null;
	connectFlag = false;
    }

    X80Pro(String ip, int port) {
	try {
	    connect(ip, port);
	} catch (Exception e) {
	    e.printStackTrace();
	}
    }
}
