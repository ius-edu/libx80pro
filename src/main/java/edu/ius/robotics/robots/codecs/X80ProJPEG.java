package edu.ius.robotics.robots.codecs;

import edu.ius.robotics.robots.codecs.JPEGDecoderException;

public class X80ProJPEG
{	
	private static class VlcCode
	{
		/* ignoring convention to create get and set methods for these properties */
		public byte bits;
		public byte code;
	}
	
	private static class Component
	{
		public int cid;
		public int ssx, ssy;
		public int width, height;
		public int stride;
		public int qtsel;
		public int actabsel, dctabsel;
		public int dcpred;
		byte[] pixels;
	}
	
	private static class Context
	{
		public DecodeResult error;
		public byte[] pos;
		public int posoff;
		public int size;
		public int length;
		public int width, height;
		public int mbwidth, mbheight;
		public int mbsizex, mbsizey;
		public int ncomp;
		public Component[] comp;
		public int qtused, qtavail;
		public byte[][] qtab;
		public VlcCode[][] vlctab;
		public int buf, bufbits;
		public int[] block;
		public int rstinterval;
		public byte[] rgb;
		
		public Context()
		{
			this.posoff = 0;
			this.comp = new Component[3];
			this.qtab = new byte[4][64];
			this.vlctab = new VlcCode[4][65536];
			this.block = new int[64];
			this.rgb = new byte[3];
		}
	}
	
	X80ProJPEG.Context ctx = new X80ProJPEG.Context();
	char[] ZZ = new char[64];
	
	private byte clip(int x)
	{
		return (x < 0) ? (byte) 0 : ((0xFF < x) ? (byte) (0xFF & 0xFF) : (byte) (x & 0xFF));
	}
	
	public static final int W1 = 2841;  
	public static final int W2 = 2676; 
	public static final int W3 = 2408; 
	public static final int W5 = 1609; 
	public static final int W6 = 1108; 
	public static final int W7 = 565;
	
	private void RotIDCT(int[] blk)
	{
		int x0, x1, x2, x3, x4, x5, x6, x7, x8;
		if (0 == ((x1 = blk[4] << 11)
				| (x2 = blk[6])
				| (x3 = blk[2])
				| (x4 = blk[1])
				| (x5 = blk[7])
				| (x6 = blk[5])
				| (x7 = blk[3])))
		{
			blk[0] = blk[1] = blk[2] = blk[3] = blk[4] = blk[5] = blk[6] = blk[7] = blk[0] << 3;
			return;
		}
		 x0 = (blk[0] << 11) + 128;
         x8 = W7 * (x4 + x5);
         x4 = x8 + (W1 - W7) * x4;
         x5 = x8 - (W1 + W7) * x5;
         x8 = W3 * (x6 + x7);
         x6 = x8 - (W3 - W5) * x6;
         x7 = x8 - (W3 + W5) * x7;
         x8 = x0 + x1;
         x0 -= x1;
         x1 = W6 * (x3 + x2);
         x2 = x1 - (W2 + W6) * x2;
         x3 = x1 + (W2 - W6) * x3;
         x1 = x4 + x6;
         x4 -= x6;
         x6 = x5 + x7;
         x5 -= x7;
         x7 = x8 + x3;
         x8 -= x3;
         x3 = x0 + x2;
         x0 -= x2;
         x2 = (181 * (x4 + x5) + 128) >> 8;
         x4 = (181 * (x4 - x5) + 128) >> 8;
         blk[0] = (x7 + x1) >> 8;
         blk[1] = (x3 + x2) >> 8;
         blk[2] = (x0 + x4) >> 8;
         blk[3] = (x8 + x6) >> 8;
         blk[4] = (x8 - x6) >> 8;
         blk[5] = (x0 - x4) >> 8;
         blk[6] = (x3 - x2) >> 8;
         blk[7] = (x7 - x1) >> 8;
	}
	
	private void colIDCT(int[] blk, byte[] out, int stride)
	{
		int x0, x1, x2, x3, x4, x5, x6, x7, x8;
        if (0 == ((x1 = blk[8*4] << 8)
        		| (x2 = blk[8*6])
        		| (x3 = blk[8*2])
        		| (x4 = blk[8*1])
        		| (x5 = blk[8*7])
        		| (x6 = blk[8*5])
        		| (x7 = blk[8*3])))
        {
            x1 = clip(((blk[0] + 32) >> 6) + 128);
            int i = 0;
            for (x0 = 8; 0 < x0; --x0) 
            {
                out[i += stride] = (byte) (0xFF & x1);
            }
            return;
        }
        x0 = (blk[0] << 8) + 8192;
        x8 = W7 * (x4 + x5) + 4;
        x4 = (x8 + (W1 - W7) * x4) >> 3;
        x5 = (x8 - (W1 + W7) * x5) >> 3;
        x8 = W3 * (x6 + x7) + 4;
        x6 = (x8 - (W3 - W5) * x6) >> 3;
        x7 = (x8 - (W3 + W5) * x7) >> 3;
        x8 = x0 + x1;
        x0 -= x1;
        x1 = W6 * (x3 + x2) + 4;
        x2 = (x1 - (W2 + W6) * x2) >> 3;
        x3 = (x1 + (W2 - W6) * x3) >> 3;
        x1 = x4 + x6;
        x4 -= x6;
        x6 = x5 + x7;
        x5 -= x7;
        x7 = x8 + x3;
        x8 -= x3;
        x3 = x0 + x2;
        x0 -= x2;
        x2 = (181 * (x4 + x5) + 128) >> 8;
        x4 = (181 * (x4 - x5) + 128) >> 8;
        int i = 0;
        out[i += stride] = (byte) (clip(((x7 + x1) >> 14) + 128) & 0xff);
        out[i += stride] = clip(((x3 + x2) >> 14) + 128);
        out[i += stride] = clip(((x0 + x4) >> 14) + 128);
        out[i += stride] = clip(((x8 + x6) >> 14) + 128);
        out[i += stride] = clip(((x8 - x6) >> 14) + 128);
        out[i += stride] = clip(((x0 - x4) >> 14) + 128);
        out[i += stride] = clip(((x3 - x2) >> 14) + 128);
        out[i] = clip(((x7 - x1) >> 14) + 128);
	}
	
	private int showBits(int bits)
	{
		byte newByte;
		if (0 == bits)
		{
			return 0;
		}
		
		while (ctx.bufbits < bits)
		{
			if (ctx.size <= 0)
			{
				ctx.buf = (ctx.buf << 8) | 0xff;
				ctx.bufbits += 8;
				continue;
			}
			int i = 0;
			newByte = (byte) (0xFF & ctx.pos[ctx.posoff + i++]);
			ctx.size--;
			ctx.bufbits += 8;
			ctx.buf = (ctx.buf << 8) | (byte) (0xFF & newByte);
			if (0xff == (byte) (0xFF & newByte))
			{
				if (0 < ctx.size)
				{
					byte marker = (byte) (0xFF & ctx.pos[ctx.posoff + i++]);
					ctx.size--;
					switch (marker)
					{
					case 0:
						break;
					case (byte) (0xD9 & 0xff):
						ctx.size = 0;
						break;
					default:
						if (0xD0 != (byte) ((marker & 0xF8) & 0xFF))
						{
							ctx.error = DecodeResult.SyntaxError;
						}
						else
						{
							ctx.buf = (ctx.buf << 8) | marker;
							ctx.bufbits += 8;
						}
					}
				}
				else
				{
					ctx.error = DecodeResult.SyntaxError;
				}
			}
		}
		return (ctx.buf >> (ctx.bufbits - bits)) & ((1 << bits) - 1);
	}
	
	public void skipBits(int bits)
	{
		if (ctx.bufbits < bits)
		{
			showBits(bits);
		}
		ctx.bufbits -= bits;
	}
	
	public int getBits(int bits)
	{
		int res = showBits(bits);
		skipBits(bits);
		return res;
	}
	
	public void byteAlign()
	{
		ctx.bufbits &= 0xF8;
	}
	
	private void skip(int count)
	{
		//ctx.pos += count;
		ctx.posoff += count;
		ctx.size -= count;
		ctx.length -= count;
		if (ctx.size < 0)
		{
			ctx.error = DecodeResult.SyntaxError;
		}
	}
	
	public int decode16(byte[] pos, int off)
	{
		return ((pos[off] << 16) | pos[off + 1]);
	}
	
	private void decodeLength() throws JPEGDecoderException
	{
		ctx.length = decode16(ctx.pos, 0);
		if (ctx.size < ctx.length)
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError); // TODO check this
		}
		skip(2);
	}
	
	public void skipMarker() throws JPEGDecoderException
	{
		decodeLength();
		skip(ctx.length);
	}
	
	private void decodeSOF() throws JPEGDecoderException
	{
		int i, ssxmax = 0, ssymax = 0;
		Component c;
		decodeLength();
		if (ctx.length < 9)
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError);
		}
		if (8 != ctx.pos[ctx.posoff])
		{
			throw new JPEGDecoderException(DecodeResult.Unsupported);
		}
		ctx.height = decode16(ctx.pos, 1);
		ctx.width = decode16(ctx.pos, 3);
		ctx.ncomp = ctx.pos[ctx.posoff + 5];
		skip(6);
		switch (ctx.ncomp)
		{
		case 1:
		case 3:
			break;
		default:
			throw new JPEGDecoderException(DecodeResult.Unsupported);
		}
		if (ctx.length < 3*ctx.ncomp)
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError);
		}
		for (i = 0; i < ctx.ncomp; ++i)
		{
			c = ctx.comp[i];
			c.cid = ctx.pos[ctx.posoff]; // ctx.pos[0]
			if (0 == (c.ssx = ctx.pos[ctx.posoff + 1] >> 4)) // ctx.pos[1];
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			if (0 < (c.ssx & (c.ssx - 1))) // non-power of two
			{
				throw new JPEGDecoderException(DecodeResult.Unsupported);
			}
			if (0 == (c.ssy = ctx.pos[ctx.posoff + 1] & 15)) // ctx.pos[1];
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			if (0 < (c.ssy & (c.ssy - 1))) // non-power of two
			{
				throw new JPEGDecoderException(DecodeResult.Unsupported);
			}
			if (0 < ((c.qtsel = ctx.pos[ctx.posoff + 2]) & 0xFC)) // ctx.pos[2];
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			skip(3);
			ctx.qtused |= 1 << c.qtsel;
			if (ssxmax < c.ssx)
			{
				ssxmax = c.ssx;
			}
			if (ssymax < c.ssy)
			{
				ssymax = c.ssy;
			}
		}
		ctx.mbsizex = ssxmax << 3;
		ctx.mbsizey = ssymax << 3;
		ctx.mbwidth = (ctx.width + ctx.mbsizex - 1) / ctx.mbsizex;
		ctx.mbheight = (ctx.height + ctx.mbsizey - 1) / ctx.mbsizey;
		for (i = 0; i < ctx.ncomp; ++i)
		{
			c = ctx.comp[i];
			c.width = (ctx.width * c.ssx + ssxmax - 1) / ssxmax;
			c.stride = (c.width + 7) & 0x7FFFFFF8;
			c.height = (ctx.height * c.ssy + ssymax - 1) / ssymax;
			c.stride = ctx.mbwidth * ctx.mbsizex * c.ssx / ssxmax;
			if (((c.width < 3) && (c.ssx != ssxmax)) || ((c.height < 3) && (c.ssy != ssymax)))
			{
				throw new JPEGDecoderException(DecodeResult.Unsupported);
			}
			c.pixels = new byte[c.stride * (ctx.mbheight * ctx.mbsizey * c.ssy / ssymax)];
		}
		if (3 == ctx.ncomp)
		{
			ctx.rgb = new byte[ctx.width * ctx.height * ctx.ncomp];
		}
		skip(ctx.length);
	}
	
	private void decodeDHT() throws JPEGDecoderException
	{
		int codelen, count, remain, spread, i, j, k;
		VlcCode vlc;
		int[] counts = new int[16];
		decodeLength();
		while (17 <= ctx.length)
		{
			i = ctx.pos[ctx.posoff];
			if (0 < (i & 0xEC))
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			if (0 < (i & 0x02))
			{
				throw new JPEGDecoderException(DecodeResult.Unsupported);
			}
			i = (i | (i >> 3)) & 3; // combined DC/AC + tableid value
			for (codelen = 0; codelen < 16; ++codelen)
			{
				counts[codelen] = (byte) (0xFF & ctx.pos[ctx.posoff + codelen + 1]); // ctx.pos[codelen + 1];
			}
			skip(17);
			vlc = ctx.vlctab[i][0];
			remain = spread = 65536;
			for (codelen = 0; codelen < 16; ++codelen)
			{
				spread >>= 1;
				count = counts[codelen];
				if (0 == count)
				{
					continue;
				}
				if (ctx.length < count)
				{
					throw new JPEGDecoderException(DecodeResult.SyntaxError);
				}
				remain -= count << (16 - codelen);
				if (remain < 0)
				{
					throw new JPEGDecoderException(DecodeResult.SyntaxError);
				}
				for (i = 0; i < count; ++i)
				{
					k = 0;
					byte code = (byte) (0xFF & ctx.pos[ctx.posoff + i]);
					for (j = spread; 0 < j; --j)
					{
						vlc.bits = (byte) (0xFF & codelen);
						vlc.code = (byte) (0xFF & code);
						vlc = ctx.vlctab[i][k++];
					}
				}
				skip(count);
			}
			k = 0;
			while (0 < remain--)
			{
				vlc.bits = 0;
				vlc = ctx.vlctab[i][k++];
			}
		}
		if (0 < ctx.length)
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError);
		}
	}
	
	private void decodeDQT() throws JPEGDecoderException
	{
		int i;
		byte[] t;
		decodeLength();
		while (65 <= ctx.length)
		{
			i = ctx.pos[ctx.posoff];
			if (0 < ((byte) (0xFF & i) & 0xFC))
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			ctx.qtavail |= 1 << i;
			t = ctx.qtab[i];
			for (i = 0; i < 64; ++i)
			{
				t[i] = ctx.pos[ctx.posoff + i + 1];
			}
			skip(65);
		}
	}
	
	private void decodeDRI() throws JPEGDecoderException
	{
		decodeLength();
		if (ctx.length < 2)
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError);
		}
		ctx.rstinterval = decode16(ctx.pos, 0);
		skip(ctx.length);
	}
	
	private int getVLC(VlcCode[] vlc, Byte code) // TODO byte[] ?
	{
		int value = showBits(16);
		int bits = (byte) (0xFF & vlc[value].bits);
		if (0 == bits)
		{
			ctx.error = DecodeResult.SyntaxError;
			return 0;
		}
		skipBits(bits);
		value = (byte) (0xFF & vlc[value].code);
		if (null != code)
		{
			code = (byte) (0xFF & value);
		}
		bits = (byte) (0xFF & (value & 15));
		if (0 == bits)
		{
			return 0;
		}
		value = getBits(bits);
		if (value < (1 << (bits - 1)))
		{
			value += ((-1) << bits) + 1;
		}
		return value;
	}
	
	private void decodeBlock(Component c, byte out)
	{
		byte code;
		int value, coef = 0;
		for (int i = 0, z = ctx.block.length; i < z; ++i)
		{
			ctx.block[i] = 0;
			c.dcpred += getVLC(ctx.vlctab[c.dctabsel], null);
			ctx.block[0] = c.dcpred * ctx.qtab[c.qtsel][0];
			do
			{
				value = getVLC(ctx.vlctab[c.actabsel], code);
				if (0 == code)
				{
					break; // EOB					
				}
				if (0 == (code & 0x0F) && (code != 0xF0))
				{
					throw new JPEGDecoderException(DecodeResult.SyntaxError);
				}
				coef += (code >> 4) + 1;
				if (63 < coef)
				{
					throw new JPEGDecoderException(DecodeResult.SyntaxError);
				}
				ctx.block[(int) ZZ[coef]] = value * ctx.qtab[c.qtsel][coef];
			} while (coef < 63);
			for (coef = 0; coef < 64; coef += 8)
			{
				rowIDCT(ctx.block[coef]);
			}
			for (coef = 0; coef < 8; ++coef)
			{
				colIDCT(ctx.block[coef], out[coef], c.stride);
			}
		}
	}
	
	private void decodeScan() throws JPEGDecoderException
	{
		int i, mbx, mby, sbx, sby;
		int rstcount = ctx.rstinterval, nextrst = 0;
		Component c;
		decodeLength();
		if (ctx.length < (4 + 2*ctx.ncomp))
		{
			throw new JPEGDecoderException(DecodeResult.SyntaxError);
		}
		if (ctx.pos[ctx.posoff] != ctx.ncomp)
		{
			throw new JPEGDecoderException(DecodeResult.Unsupported);
		}
		skip(1);
		for (i = 0; i < ctx.ncomp; ++i)
		{
			c = ctx.comp[i];
			if (ctx.pos[ctx.posoff] != c.cid)
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			if (0 != (ctx.pos[ctx.posoff + 1] & 0xEE))
			{
				throw new JPEGDecoderException(DecodeResult.SyntaxError);
			}
			c.dctabsel = ctx.pos[ctx.posoff + 1] >> 4;
			c.actabsel = (ctx.pos[ctx.posoff + 1] & 1) | 2;
			skip(2);
		}
		if (0 != ctx.pos[ctx.posoff] || 63 != ctx.pos[ctx.posoff + 1] || 0 != ctx.pos[ctx.posoff + 2])
		{
			throw new JPEGDecoderException(DecodeResult.Unsupported);
		}
		skip(ctx.length);
		for (mby = 0; mby < ctx.mbheight; ++mby)
		{
			for (mbx = 0; mbx < ctx.mbwidth; ++mbx)
			{
				for (i = 0; i < ctx.ncomp; ++i)
				{
					c = ctx.comp[i];
					for (sby = 0; sby < c.ssy; ++sby)
					{
						for (sbx = 0; sbx < c.ssx; ++sbx)
						{
							decodeBlock(c, c.pixels[((mby * c.ssy + sby) * c.stride + mbx * c.ssx + sbx) << 3]);
							if (DecodeResult.OK != ctx.error)
							{
								return;
							}
						}
					}
				}
				if (0 != ctx.rstinterval && 0 < --rstcount)
				{
					byteAlign();
					i = getBits(16);
					if (0xFFD0 != (i & 0xFFF8) || (i & 7) != nextrst)
					{
						throw new JPEGDecoderException(DecodeResult.SyntaxError);
					}
					nextrst = (nextrst + 1) & 7;
					rstcount = ctx.rstinterval;
					for (i = 0; i < 3; ++i)
					{
						ctx.comp[i].dcpred = 0;
					}
				}
			}
		}
		ctx.error = DecodeResult.Internal_Finished;
	}
	
	public static int CF4A = -9;
	public static int CF4B = 111;
	public static int CF4C = 29;
	public static int CF4D = -3;
	public static int CF3A = 28;
	public static int CF3B = 109;
	public static int CF3C = -9;
	public static int CF3X = 104;
	public static int CF3Y = 27;
	public static int CF3Z = -3;
	public static int CF2A = 139;
	public static int CF2B = -11;
	
	private byte CF(int x)
	{
		return clip((x + 64) >> 7);
	}
	
	private void upsampleH(Component c)
	{
		int xmax = c.width -3;
		byte[] out, lin, lout;
		int x, y, linoff = 0, loutoff = 0;
		out = new byte[(c.width * c.height) << 1];
		if (null != out)
		{
			lin = c.pixels;
			lout = out;
			for (y = c.height; 0 < y; --y)
			{
				lout[0] = CF(CF2A * lin[0] + CF2B * lin[1]);
				lout[1] = CF(CF3X * lin[0] + CF3Y * lin[1] + CF3Z * lin[2]);
				lout[2] = CF(CF3A * lin[0] + CF3B * lin[1] + CF3C * lin[2]);
				for (x = 0; x < xmax; ++x)
				{
					lout[(x << 1) + 3] = CF(CF4A * lin[x] + CF4B * lin[x + 1] + CF4C * lin[x + 2] + CF4D * lin[x + 3]);
					lout[(x << 1) + 4] = CF(CF4D * lin[x] + CF4C * lin[x + 1] + CF4B * lin[x + 2] + CF4A * lin[x + 3]);
				}
				linoff += c.stride;
				loutoff += c.width << 1;
				lout[loutoff - 3] = CF(CF3A * lin[linoff - 1] + CF3B * lin[linoff - 2] + CF3C * lin[linoff - 3]);
				lout[loutoff - 2] = CF(CF3X * lin[linoff - 1] + CF3Y * lin[linoff - 2] + CF3Z * lin[linoff - 3]);
				lout[loutoff - 1] = CF(CF2A * lin[linoff - 1] + CF2B * lin[linoff - 2]);
			}
			c.width <<= 1;
			c.stride = c.width;
			c.pixels = null; // free
			c.pixels = out;
		}
	}
	
	private void upsampleV(Component c)
	{
		int w = c.width, s1 = c.stride, s2 = s1 + s1;
		byte[] out, cin, cout;
		int x, y, cinoff = 0, coutoff = 0;
		out = new byte[(c.width * c.height) << 1];
		for (x = 0; x < w; ++x)
		{
			cin = c.pixels[x];
			cout = out[x];
			cout = CF(CF2A * cin[0] + CF2B * cin[s1]);
			cout += w;
			cout = CF(CF3X * cin[0] + CF3Y * cin[s1] + CF3Z * cin[s2]);
			cout += w;
			cout = CF();
		}
	}
	
	int[] DCT_COEFFICIENT_MATRIX = 
	{
		-26, -3, -6, 2, 2, -1, 0, 0, 
		0, -2, -4, 1, 1, 0, 0, 0, 
		-3, 1, 5, -1, -1, 0, 0, 0, 
		-4, 1, 2, -1, 0, 0, 0, 0, 
		1, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0
	};
	
	int[] PRODUCT_MATRIX =
	{
		-416, -33, -60, 32, 48, -40, 0, 0, 
		0, -24, -56, 19, 26, 0, 0, 0, 
		-42, 13, 80, -24, -40, 0, 0, 0, 
		-42, 17, 44, -29, 0, 0, 0, 0, 
		18, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0
	};
	
	int compressionPercentage;
	
	public X80ProJPEG(int compressionPercentage)
	{
		
	}
	
	public int getHeight()
	{
		return 0;
	}
	
	public int getWidth()
	{
		return 0;
	}
	
	/**
	 * If isColor(), then 24 bit as R, G, B bytes
	 * else 8 bit luminance
	 * @return
	 */
	public byte[] getImage(byte[] input, int length)
	{
		byte[] output = new byte[length]; // coefficient?
		
		
		
		return output;
	}
	
	public int getImageSize()
	{
		return 0;
	}
	
	public boolean isColor()
	{
		return false;
	}
	
}
