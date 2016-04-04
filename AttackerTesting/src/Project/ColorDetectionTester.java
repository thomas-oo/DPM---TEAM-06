package Project;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * Color Detection Tester
 *
 */
public class ColorDetectionTester {
	
	//initializing EV3ColorSensor
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private static SensorMode colorValue = colorSensor.getMode("RGB");
	private static int sampleSize = colorValue.sampleSize();
	private static float[] colorData = new float[sampleSize];
	
	/**
	 * starting the BallColorDetector class
	 * @param args
	 */
	public static void main(String[]args)
	{
		BallColorDetector detector = new BallColorDetector(colorSensor, colorData, colorValue);
		detector.start();
		
		Button.waitForAnyPress();
		System.exit(0);
	}
}
