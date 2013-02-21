package edu.ius.robotics.robots.boards;

/* The following is taken pretty much directly from the 
 * PMB5010 serial protocol documentation 
 */

public class PMB5010
{
	public static class ADPCM 
	{
		public static int[] indexTable = 
		{
			-1, -1, -1, -1, 2, 4, 6, 8, 
			-1, -1, -1, -1, 2, 4, 6, 8
		};
		
		public static int[] stepsizeTable = 
		{
			7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 
			19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 
			50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 
			130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 
			337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 
			876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 
			2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 
			5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 
			15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
		};
		
		public static class ADPCMState 
		{
			short valprev; /* previous output value */
			char index; /* index into step size table */
			
			public short getValPrev()
			{
				return this.valprev;
			}
			
			public void setValPrev(int valprev)
			{
				this.valprev = (short)valprev;
			}
			
			public char getIndex()
			{
				return this.index;
			}
			
			public void setIndex(int index)
			{
				this.index = (char)index;
			}
		};
		
		ADPCMState adpcmState;
		
		public ADPCM()
		{
			adpcmState.setValPrev(0);
			adpcmState.setIndex(0);
		}
		
		// lpInData points to msg[6], nLen = Data_Len
		public short[] dealWithAudio(char[] inData)
		{
			return adpcmDecode(inData);
		}
		
		public short[] adpcmDecode(char[] inData)
		{
			short[] outData = new short[10];
			
			int sign;
			int delta;
			int step;
			int valPred;
			int vpDiff;
			int index;
			int inputBuffer;
			int bufferStep;
			
			
			
			return outData;
		}
	}
	
	/* Packet Info */
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int DID_OFFSET = 4;
	
	/* Start transmission, End transmission */
	public static final byte STX0 = 94; // 0x5e
	public static final byte STX1 = 2; // 0x02
	public static final byte ETX0 = 94; // 0x5e
	public static final byte ETX1 = 13; // 0x13
	
	/* RID */
	public static final byte RID_HOST = 0x00;
	public static final byte RID_PMB5010 = 0x08;
	
	/* RESERVED */
	public static final byte RESERVED = 0x00;
	
	/* FLAG */
	public static final byte FLAG_VALUE = 0x06;
	
	/* DID Listing */
	public static final byte AUDIO_PACKET = 0x0A;
	public static final byte START_AUDIO_RECORDING = 0x0B;
	public static final byte STOP_AUDIO_RECORDING = 0x0C;
	public static final byte START_AUDIO_PLAYBACK = 0x0D;
	public static final byte STOP_AUDIO_PLAYBACK = 0x0E;
	public static final byte TAKE_PHOTO = 0x20;
	
    /*
     * calcCRC method comes directly from PMB5010 Protocol documentation.
     */
    public static byte calcCRC(byte[] buf) 
    {
		byte shift_reg, sr_lsb, data_bit, v;
		byte fb_bit;
		int z;
		shift_reg = 0; // initialize the shift register
		z = buf.length - 5;
		
		for (int i = 0; i < z; ++i) 
		{
		    v = (byte) (buf[2 + i]); // start from RID
		    
		    // for each bit
		    for (int j = 0; j < 8; ++j) 
		    {
				// isolate least sign bit
				data_bit = (byte) ((v & 0x01) & 0xff);
				sr_lsb = (byte) ((shift_reg & 0x01) & 0xff);
				// calculate the feed back bit
				fb_bit = (byte) (((data_bit ^ sr_lsb) & 0x01) & 0xff);
				shift_reg = (byte) ((shift_reg & 0xff) >>> 1);
				
				if (fb_bit == 1)
				{
				    shift_reg = (byte) ((shift_reg ^ 0x8C) & 0xff);
				}
				
				v = (byte) ((v & 0xff) >>> 1);
		    }
		}
		
		return shift_reg;
    }
    
    public static byte[] startAudioRecording(byte voiceSegmentLength)
    {
    	byte[] msg = new byte[10];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = START_AUDIO_RECORDING;
    	msg[5] = 1; // len
    	msg[6] = voiceSegmentLength;
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] stopAudioRecording(byte voiceSegmentLength)
    {
    	byte[] msg = new byte[9];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = RESERVED;
    	msg[4] = STOP_AUDIO_RECORDING;
    	msg[5] = 0;
    	msg[6] = calcCRC(msg);
    	msg[7] = ETX0;
    	msg[8] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] startAudioPlayback(short sampleLength)
    {
    	byte[] msg = new byte[10];
    	
    	int length = 4;
    	
    	if (sampleLength < 16000)
    	{
    		length = 1;
    	}
    	else if (sampleLength < 16000*20)
    	{
    		length = 2;
    	}
    	else if (sampleLength < 16000*32)
    	{
    		length = 3;
    	}
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = 0; // TODO SEQ
    	msg[4] = START_AUDIO_PLAYBACK;
    	msg[5] = 1;
    	msg[6] = (byte) (length & 0xff);
    	msg[7] = calcCRC(msg);
    	msg[8] = ETX0;
    	msg[9] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] stopAudioPlayback()
    {
    	byte[] msg = new byte[9];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = 0;
    	msg[4] = STOP_AUDIO_PLAYBACK;
    	msg[5] = 0;
    	msg[6] = calcCRC(msg);
    	msg[7] = ETX0;
    	msg[8] = ETX1;
    	
    	return msg;
    }
    
    public static byte[] continueAudioPlayback(int length)
    {
    	byte[] msg = new byte[];
    	
    	msg[0] = STX0;
    	msg[1] = STX1;
    	msg[2] = RID_PMB5010;
    	msg[3] = 0;
    	msg[4] = AUDIO_PACKET;
    	msg[5] = length;
    	msg[6] 
    	
    	
    	return msg;
    }
}
