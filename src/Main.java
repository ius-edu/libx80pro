import java.io.IOException;
import java.util.Enumeration;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;


public class Main
{
	public static void main(String[] args) throws IOException
	{
		// TODO Auto-generated method stub
		ZipFile z = new ZipFile("/home/colton/workspace/Integration_Jars/com.myhradmin.multiclassloader.jar");
		Enumeration<? extends ZipEntry> entries = z.entries();
		
		while(entries.hasMoreElements())
		{
			ZipEntry e = entries.nextElement();
			if(!e.isDirectory()) {
				System.out.println(e.getName());
			}
		}
		z.close();
	}
}
