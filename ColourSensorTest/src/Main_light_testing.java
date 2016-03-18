
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main_light_testing {
	static PrintStream writer = System.out;
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	@SuppressWarnings("resource")
	public static void main(String[] args) throws InterruptedException, FileNotFoundException,UnsupportedEncodingException{
		int button_select;
		PrintStream writer = null;
		writer = new PrintStream("data.csv","UTF-8");
		int current=0;
		int last=0;
		
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		final SampleProvider colorValue = colorSensor.getMode("ColorID");			// colorValue provides samples from this instance
		//final float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
		
		button_select = Button.waitForAnyPress();
		while(button_select != Button.ID_LEFT && button_select != Button.ID_RIGHT);
			if(button_select == Button.ID_LEFT)
			{
			
			LightSensor lightsensor = new LightSensor(colorValue);	
			lightsensor.start();
			last=lightsensor.lightreading();
			Thread.sleep(1000);
			try {
//				leftMotor.setSpeed(100);
//				rightMotor.setSpeed(100);
//				leftMotor.forward();
//				rightMotor.forward();
				
				while(true){
					
					current=lightsensor.lightreading();
				//	int diff = last-current;
					
					//without any filter
					System.out.print(String.format("%d:%d%n",Math.round(System.currentTimeMillis()),current));
					writer.println(current);
//					if(Math.abs(diff)>3){
//						Sound.beep();
//					}
//					
//					last=current;
					Thread.sleep(100);
				}
				
				
				
			} finally  {
				// TODO: handle exception
				writer.close();
				writer.close();
			}
				
			}
			// Stop the program.
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);	
	}
	
}
