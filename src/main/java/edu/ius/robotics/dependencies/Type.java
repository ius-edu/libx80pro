package edu.ius.robotics.dependencies;

public class Type
{
	public static int sizeOfByte = Byte.SIZE >> 3;
	public static int sizeOfShort = Short.SIZE >> 3;
	public static int sizeOfInt = Integer.SIZE >> 3;
	public static int sizeOfLong = Long.SIZE >> 3;
	
	public static int sizeof(byte b) { return sizeOfByte; } 
	public static int sizeof(short s) { return sizeOfShort; }
	public static int sizeof(int i) { return sizeOfInt; }
	public static int sizeof(long l) { return sizeOfLong; }
	
	public static int unsigned(byte b) { return b & 0xFF; }
	public static int unsigned(short s) { return s & 0xFFFF; }
	public static long unsigned(int i) { return i & 0xFFFFFFFF; }
	
	public static boolean isEven(int i) { return 0 == i % 2; }
	public static boolean isOdd(int i) { return 1 == i % 2; }
}
