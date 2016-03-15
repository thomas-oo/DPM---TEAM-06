package Robot;

import Robot.Navigator;
import Robot.Odometer;
import Robot.OdometryDisplay;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;



public class Main 
{
	
	//creating instances of motors for use
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor headMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");	
	
	//specifications about robot, global and should be accessible.
	public static double WHEEL_RADIUS = 2.1; //measure
	public static double TRACK = 15.7;
	public static double LS_DIST = 12.5;
	public static int bandCenter = 20;
	public static int bandWidth = 2;
	public static int motorLow = 100;
	public static int motorHigh = 200;
	
	//classes main will rely on
	public static Navigator nav;
	public static Odometer odometer;
	public static OdometryDisplay odometryDisplay;
	
	@SuppressWarnings("resource")
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");
	public static float[] usData = new float[usValue.sampleSize()];
	
	public static SensorModes colorSensor = new EV3ColorSensor(colorPort);
	public static SampleProvider colorValue = colorSensor.getMode("Red");
	public static float[] colorData = new float[colorValue.sampleSize()];		
	public static void main(String[] args) 
	{
		
	}

}
