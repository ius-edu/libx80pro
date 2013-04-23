package edu.ius.robotics.robots.x80pro;

import static edu.ius.robotics.convenience.Type.unsigned;

import java.nio.ByteBuffer;

public class DataPackage 
{
	public static final int STX0_OFFSET = 0;
	public static final int STX1_OFFSET = 1;
	public static final int RID_OFFSET = 2;
	public static final int SEQ_OFFSET = 3;
	public static final int DID_OFFSET = 4;
	public static final int DATA_LENGTH_OFFSET = 5;
	public static final int DATA_OFFSET = 6;
	public static final int CHECKSUM_RELATIVE_OFFSET = 0;
	public static final int ETX0_RELATIVE_OFFSET = 1;
	public static final int ETX1_RELATIVE_OFFSET = 2;
	public static final int HEADER_LENGTH = 6;
	public static final int FOOTER_LENGTH = 3;
	public static final int METADATA_SIZE = 9;
	public static final int MAX_DATA_SIZE = 0xFF;
	public static final int ADDITIONAL_ADPCM_DATA_LENGTH = 5;
	public static final int STX0 = 0x5E;
	public static final int STX1 = 0x02;
	public static final int ETX0 = 0x5E;
	public static final int ETX1 = 0x0D;
	public static final int SEQ_BEGIN = 0x00;
	public static final int SEQ_END = 0xFF;
	
	public int length;
	
	private int offset;
	private int dataLength;
	private int footerOffset;
	private int checksumOffset;
	private int etx0offset;
	private int etx1offset;
	
	private ByteBuffer repHeader;
	private ByteBuffer repData;
	private ByteBuffer repFooter;
	
	private String robotIP;
	private int robotPort;
	private boolean isReady;
	
	public void clear()
	{
		robotIP = "";
		offset = dataLength = length = robotPort = 0;
		footerOffset = HEADER_LENGTH;
		repHeader.clear();
		repData.clear();
		repFooter.clear();
		isReady = false;
	}
	
	public DataPackage()
	{
		repHeader = ByteBuffer.allocate(HEADER_LENGTH);
		repData = ByteBuffer.allocate(MAX_DATA_SIZE + ADDITIONAL_ADPCM_DATA_LENGTH);
		repFooter = ByteBuffer.allocate(FOOTER_LENGTH);
		clear();
	}
	
	private byte doGet(int i)
	{
		byte result = -1;
		
		if (0 <= i && i < HEADER_LENGTH)
		{
			result = repHeader.get(i);
		}
		else if (HEADER_LENGTH <= i && i < footerOffset)
		{
			result = repData.get(i - HEADER_LENGTH);
		}
		else if (footerOffset <= i && i < length)
		{
			result = repFooter.get(i - footerOffset);
		}
		
		return result;
	}
	
	private short doGetShort(int i)
	{
		short result = -1;
		
		if (0 <= i && i < HEADER_LENGTH)
		{
			result = repHeader.getShort(i);
		}
		else if (HEADER_LENGTH <= i && i < footerOffset)
		{
			result = repData.getShort(i - HEADER_LENGTH);
		}
		else if (footerOffset <= i && i < length)
		{
			result = repFooter.getShort(i - footerOffset);
		}
		
		return result;
	}
	
	private int doGetInt(int i)
	{
		int result = -1;
		
		if (0 <= i && i < HEADER_LENGTH)
		{
			result = repHeader.getInt(i);
		}
		else if (HEADER_LENGTH <= i && i < footerOffset)
		{
			result = repData.getInt(i - HEADER_LENGTH);
		}
		else if (footerOffset <= i && i < length)
		{
			result = repFooter.getInt(i - footerOffset);
		}
		
		return result;
	}
	
	private byte doGetCheckSum(int offset, int length)
	{
		byte shift_reg, sr_lsb, data_bit, v, fb_bit;
		
		shift_reg = 0; // initialize the shift register
		for (int i = offset, z = length; i < z; ++i)
		{
			v = doGet(i);
			for (int j = 0; j < 8; ++j) // for each bit
			{
				data_bit = (byte) (v & 0x01); // isolate least sign bit
				sr_lsb = (byte) (shift_reg & 0x01);
				fb_bit = (byte) ((data_bit ^ sr_lsb) & 0x01); // calculate feedback bit
				shift_reg = (byte) (shift_reg >>> 1);
				if (1 == fb_bit)
				{
					shift_reg = (byte) (shift_reg ^ 0x8C);
				}
				v = (byte) (v >>> 1);
			}
		}
		return (byte) shift_reg;
	}
	
	// From the PMS5005 and PMB5010 protocol documentation
	public byte getCheckSum()
	{
		return doGetCheckSum(RID_OFFSET, footerOffset);
	}
	
	public byte getChecksum(int offset, int length)
	{
		return doGetCheckSum(offset, length);
	}
	
	public void print()
	{
		System.out.println("bufferLength: " + length);
		System.out.println("dataLength: " + dataLength);
		System.out.print("data: ");
		
		for (int i = 0; i < footerOffset; ++i)
		{
			System.out.print(repData.get(i) + " ");
		}
		
		System.out.println();
		System.out.println("robotIP: " + robotIP);
		System.out.println("robotPort: " + robotPort);
	}
	
	public boolean isReady()
	{
		return isReady;
	}
	
	public byte get(int offset)
	{
		return doGet(offset);
	}
	
	public short getShort(int offset)
	{
		return doGetShort(offset);
	}
	
	public int getInt(int offset)
	{
		return doGetInt(offset);
	}
	
	public ByteBuffer getData()
	{
		return repData;
	}
	
	public int getDataLength()
	{
		return dataLength;
	}
	
	public void setRobotIP(String ip)
	{
		this.robotIP = ip;
	}
	
	public String getRobotIP()
	{
		return robotIP;
	}
	
	public void setRobotPort(int port)
	{
		this.robotPort = port;
	}
	
	public int getRobotPort()
	{
		return robotPort;
	}
	
	public int getOffset()
	{
		return offset;
	}
	
	private void doPrintHeader()
	{
		//System.err.print("DEBUG: pkg header: ");
		
		for (int i = 0; i < DataPackage.HEADER_LENGTH; ++i)
		{
			//System.err.printf("%2x ", unsigned(repHeader.get(i)));
		}
		
		//System.err.println();
	}
	
	private void doReadInHeader(byte[] message, int messageOffset, int messageLength)
	{
		if (messageOffset + offset < messageLength && STX0_OFFSET == offset)
		{
			repHeader.put(STX0_OFFSET, message[messageOffset + offset]);
			if (STX0 != message[messageOffset + offset]) 
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] STX0 isn't where it is expected to be: %2x", message[messageOffset + offset]);
				//System.err.println();
			}
			else
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] Found STX0: %2x", message[messageOffset + offset]);
				//System.err.println();
			}
			++offset;
		}
		
		if (messageOffset + offset < messageLength && STX1_OFFSET == offset)
		{
			repHeader.put(STX1_OFFSET, message[messageOffset + offset]);
			if (STX1 != message[messageOffset + offset])
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] STX1 isn't where it is expected to be: %2x", message[messageOffset + offset]);
				//System.err.println();
			}
			else
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] Found STX1: %2x", message[messageOffset + offset]);
				//System.err.println();
			}
			++offset;
		}
		
		if (messageOffset + offset < messageLength && RID_OFFSET == offset)
		{
			repHeader.put(RID_OFFSET, message[messageOffset + offset]);
			//System.err.printf("DEBUG: pkg[" + offset + "] Found RID: %2x", message[messageOffset + offset]);
			//System.err.println();
			++offset;
		}
		
		if (messageOffset + offset < messageLength && DataPackage.SEQ_OFFSET == offset)
		{
			repHeader.put(DataPackage.SEQ_OFFSET, message[messageOffset + offset]);
			//System.err.printf("DEBUG: pkg[" + offset + "] Found SEQ: %2x", message[messageOffset + offset]);
			//System.err.println();
			++offset;
		}
		
		if (messageOffset + offset < messageLength && DataPackage.DID_OFFSET == offset)
		{
			repHeader.put(DataPackage.DID_OFFSET, message[messageOffset + offset]);
			//System.err.printf("DEBUG: pkg[" + offset + "] Found DID: %2x", message[messageOffset + offset]);
			//System.err.println();
			++offset;
		}
		
		if (messageOffset + offset < messageLength && DATA_LENGTH_OFFSET == offset)
		{
			repHeader.put(DATA_LENGTH_OFFSET, message[messageOffset + offset]);
			// if ADPCM, then +5?
			dataLength = unsigned(message[messageOffset + offset]);
			footerOffset = HEADER_LENGTH + dataLength;
			//System.err.println("footerOffset: " + footerOffset);
			checksumOffset = footerOffset + CHECKSUM_RELATIVE_OFFSET;
			etx0offset = footerOffset + ETX0_RELATIVE_OFFSET;
			etx1offset = footerOffset + ETX1_RELATIVE_OFFSET;
			//System.err.println("DEBUG: pkg[" + offset + "] Found DATA LENGTH: " + dataLength);
			++offset;
		}
	}
	
	private void doPrintData()
	{
		//System.err.print("DEBUG: pkg data: ");
		
		for (int i = 0; i < dataLength; ++i)
		{
			//System.err.printf("%2x ", unsigned(repData.get(i)));
		}
		
		//System.err.println();
	}
	
	private void doReadInData(byte[] message, int messageOffset, int messageLength)
	{
		if (messageOffset + offset < messageLength && DATA_OFFSET <= offset && offset < footerOffset) // ADPCM?
		{
			//System.err.print("DEBUG: pkg[" + offset + "] Found DATA: ");
			while (messageOffset + offset < messageLength && offset < footerOffset)
			{
				repData.put(message[messageOffset + offset]);
				//System.err.printf("%2x ", message[messageOffset + offset]);
				++offset;
			}
			//System.err.println();
			if (messageLength <= messageOffset + offset)
			{
				//System.err.println("DEBUG: Overflow! Package overflowed packet in data region. Previous byte: ");
				//System.err.printf("pkg.data[" + (offset - 1) + "]: %2x", unsigned(repData.get(offset - 1)));
				//System.err.println();
			}
		}
	}
	
	private void doPrintFooter()
	{
		//System.err.print("DEBUG: pkg footer: ");
		
		for (int i = 0; i < FOOTER_LENGTH; ++i)
		{
			//System.err.printf("%2x ", unsigned(repFooter.get(i)));
		}
		
		//System.err.println();
	}
	
	private void doReadInFooter(byte[] message, int messageOffset, int messageLength)
	{
		if (messageOffset + offset < messageLength && checksumOffset == offset)
		{
			repFooter.put(message[messageOffset + offset]);
			if (unsigned(message[messageOffset + offset]) != unsigned(doGetCheckSum(RID_OFFSET, footerOffset)))
			{
				//System.err.println("DEBUG: Incorrect package checksum");
				//System.err.printf("DEBUG: Expected: %2x", unsigned(doGetCheckSum(RID_OFFSET, footerOffset)));
				//System.err.println();
				//System.err.printf("DEBUG: Read: %2x", unsigned(message[messageOffset + offset]));
				//System.err.println();
			}
			else
			{
				//System.err.printf("DEBUG: Correct package checksum: %2x", doGetCheckSum(RID_OFFSET, footerOffset));
			}
			++offset;
		}
		
		if (messageOffset + offset < messageLength && etx0offset == offset)
		{
			repFooter.put(message[messageOffset + offset]);
			if (ETX0 != unsigned(message[messageOffset + offset]))
			{
				//System.err.printf("DEBUG: ETX0 isn't where it is expected to be (found %2x)", unsigned(message[messageOffset + offset]));
				//System.err.println();
			}
			else
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] Found ETX0: %2x", unsigned(message[messageOffset + offset]));
				//System.err.println();
			}
			++offset;
		}
		
		if (messageOffset + offset < messageLength && etx1offset == offset)
		{
			repFooter.put(message[messageOffset + offset]);
			
			if (ETX1 != unsigned(message[messageOffset + offset]))
			{
				//System.err.printf("DEBUG: ETX1 isn't where it is expected to be (found %2x)", unsigned(message[messageOffset + offset]));
				//System.err.println();
			}
			else
			{
				//System.err.printf("DEBUG: pkg[" + offset + "] Found ETX1 (Good package): %2x", unsigned(message[messageOffset + offset]));
				//System.err.println();
			}
			
			++offset;
			length = offset;
			//System.err.println("DEBUG: " + length + " bytes read total");
			isReady = true;
		}
	}
	
	private void doPrintPackage()
	{
		//System.err.print("DEBUG: Parts Package: ");
		for (int i = 0; i < HEADER_LENGTH; ++i)
		{
			//System.err.printf("%2x ", repHeader.get(i));
		}
		
		for (int i = 0; i < dataLength; ++i)
		{
			//System.err.printf("%2x ", repData.get(i));
		}
		
		for (int i = 0; i < FOOTER_LENGTH; ++i)
		{
			//System.err.printf("%2x ", repFooter.get(i));
		}
		//System.err.println();
		
		//System.err.print("DEBUG: Whole Package: ");
		for (int i = 0; i < length; ++i)
		{
			//System.err.printf("%2x ", doGet(i));
		}
		//System.err.println();
	}
	
	public int read(byte[] message, int messageOffset, int messageLength)
	{
		if (messageOffset < messageLength) doReadInHeader(message, messageOffset, messageLength);
		doPrintHeader();
		if (messageOffset < messageLength) doReadInData(message, messageOffset, messageLength);
		doPrintData();
		if (messageOffset < messageLength) doReadInFooter(message, messageOffset, messageLength);
		doPrintFooter();
		
		doPrintPackage();
		
		return offset; // length
	}
}
