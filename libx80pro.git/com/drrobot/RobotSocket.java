
package com.drrobot;

import java.io.*;
import java.net.*;

/**
 *
 * @author Bo Wei
 * nearly original, but modified by Jesse Riddle
 */
public class RobotSocket extends Thread {

    private DatagramSocket sock = null;
    private String robotip;
    private int robotport;
    private DatagramPacket rxpkt = null;
    private DatagramPacket txpkt = null;
    private InetAddress server = null;
    private byte[] rxbuf;
    private byte[] txbuf;
    private int[] tmp;
    private ByteArrayInputStream bin = null;
    private DataInputStream din = null;
    private DataOutputStream out = null;
    private InputStream in = null;
    private BufferedReader reader = null;
    private String strCommand = "";
    public int[] EncoderPos = { 0, 0 };
    public double[] IRDis = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 };
    //motor sensor 
    public int[] EncoderSpeed = { 0, 0 };
    public double[] MotorCurrent = { 0.0, 0.0 };
    //custom sensor data
    public int[] customAD = { 0, 0, 0, 0, 0, 0, 0, 0 };
    public int customIO = 0;
    //standard sensor data
    public double[] USDis = { 0, 0, 0, 0, 0, 0 };
    public int[] HumanAlarm = { 0, 0 };
    public int[] HumanMotion = { 0, 0 };
    public int IRRange = 0;
    public double BoardVol = 0;
    public double DCMotorVol = 0;

    /** Creates a new instance of robotSocket */
    public RobotSocket(String robotIP, int robotPort) {
        this.robotip = robotIP;
        this.robotport = robotPort;
        try {
            server = InetAddress.getByName(robotip);
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(3);
            return;
        }
        //socket connect
        try {
            sock = new DatagramSocket();
            sock.setSoTimeout(20);
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(3);
            return;
        }
        rxbuf = new byte[1024];
        rxpkt = new DatagramPacket(rxbuf, rxbuf.length);

        txbuf = new byte[256];
        txpkt = new DatagramPacket(txbuf, txbuf.length, server, robotport);
        try {
            sock.send(txpkt);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        try {
            sock.setSoTimeout(10);
            sock.receive(rxpkt);
        } catch (IOException e) {
            e.printStackTrace();
        }
        int z = rxpkt.getLength();
        if (z != 0) {
            //decode here
            tmp = new int[z];
            for (int i = 0; i < z; ++i) {
                tmp[i] = (int) (rxbuf[i] & 0xff);
            }
            if ((tmp[0] == 94) && (tmp[1] == 2) && 
		(tmp[z - 2] == 94) && (tmp[z - 1] == 13)) {
                //here is a whole package
                //first motor sensor data, please refere protocol
                if (tmp[4] == 123) {
                    EncoderPos[0] = tmp[5 + 25] + (tmp[5 + 26])*256;
                    EncoderSpeed[0] = tmp[5 + 27] + (tmp[5 + 28])*256;
                    EncoderPos[1] = tmp[5 + 29] + (tmp[5 + 30])*256;
                    EncoderSpeed[1] = tmp[5 + 31] + (tmp[5 + 32])*256;

                    MotorCurrent[0] = (double) 
			(tmp[5 + 13] + (tmp[5 + 14])*256)/728.0;
                    MotorCurrent[1] = (double) 
			(tmp[5 + 15] + (tmp[5 + 16])*256)/728.0;
                } else if (tmp[4] == 124) {
                    //custom sensor data
                    for (int i = 0; i < 8; i++) {
                        customAD[i] = tmp[6 + 2*i] + tmp[6 + 2*i + 1]*256;
                    }
                    customIO = tmp[5 + 17];
                } else if (tmp[4] == 125) {
                    //standard sensor
                    for (int i = 0; i < 6; ++i) {
                        USDis[i] = (double) (tmp[6 + i])/100.0;
                    }
                    HumanAlarm[0] = tmp[5 + 7] + tmp[5 + 8]*256;
                    HumanMotion[0] = tmp[5 + 9] + tmp[5 + 10]*256;
                    HumanAlarm[1] = tmp[5 + 11] + tmp[5 + 12]*256;
                    HumanMotion[1] = tmp[5 + 13] + tmp[5 + 14]*256;
                    IRRange = (tmp[5 + 25] + tmp[5 + 26]*256);
                    BoardVol = (double) 
			(tmp[5 + 31] + tmp[5 + 32]*256)/4095.0*9.0;
                    DCMotorVol = (double)
			(tmp[5 + 33] + tmp[5 + 34]*256)/4095.0*24.0;
                }
            }
            IRDis[0] = AD2Dis(IRRange);
            for (int i = 1; i < 7; i++) {
                IRDis[i] = AD2Dis(customAD[i + 1]);
            }
            rxpkt.setLength(rxbuf.length);
        }

    }

    public void sendCommand(byte[] cmd) {
        System.arraycopy(cmd, 0, txbuf, 0, cmd.length);
        txpkt = new DatagramPacket(txbuf, txbuf.length, server, robotport);
        try {
            sock.send(txpkt);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private double AD2Dis(int ADValue) {
        double Dis = 0;
        if ((ADValue > 4095) || (ADValue <= 0)) {
            Dis = 0;
        } else {
            Dis = 21.6 / ((double) (ADValue) * 3.0 / 4028 - 0.17);
            if (Dis > 80) {
                Dis = 0.81;
            } else if ((Dis < 10)) {
                Dis = 0.09;
            } else {
                Dis = Dis / 100.0;
            }
        }
        return Dis;
    }
}
