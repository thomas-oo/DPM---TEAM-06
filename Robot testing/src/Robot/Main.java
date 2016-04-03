package Robot;

import java.io.IOException;
import java.util.HashMap;

import Robot.Navigator;
import Robot.Odometer;
import Robot.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import wifi.WifiConnection;

public class Main 
{
	
	//creating instances of motors for use
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3MediumRegulatedMotor headMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	public static final Port usPort = LocalEV3.get().getPort("S3");
	public static final Port leftColorPort = LocalEV3.get().getPort("S4");
	public static final Port rightColorPort = LocalEV3.get().getPort("S2");
	
	
	//specifications about robot, global and should be accessible.
	public static double WHEEL_RADIUS = 2.03; //measure
	public static double TRACK = 18.0;
	public static double LS_DIST = 4.5;
	public static int wallDist = 20;
	public static int bandCenter = 20;
	public static int bandWidth = 2;
	//motorlow and high used for wall following.
	public static int motorLow = 100;
	public static int motorHigh = 180;
	
	// parameters received from WiFi
	public static int startingCorner = 1;
	public static int role; // attacker is 0, defender is 1
	public static int goalWidth;
	public static int defenderLine;
	public static int forwardLine;
	public static int lowerLeftX;
	public static int lowerLeftY;
	public static int upperRightX;
	public static int upperRightY;
	public static int ballColour;
	
	//classes main will rely on
	public static Odometer odometer;
	public static OdometryDisplay odometryDisplay;
	public static Navigator nav;
	
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");
	public static float[] usData = new float[usValue.sampleSize()];
	
	public static SensorModes leftColorSensor = new EV3ColorSensor(leftColorPort);
	public static SampleProvider leftColorValue = leftColorSensor.getMode("Red");
	public static float[] leftColorData = new float[leftColorValue.sampleSize()];		
	
	public static SensorModes rightColorSensor = new EV3ColorSensor(rightColorPort);
	public static SampleProvider rightColorValue = rightColorSensor.getMode("Red");
	public static float[] rightColorData = new float[rightColorValue.sampleSize()];
	
	// WIFI Class 
	private static final String SERVER_IP = "192.168.0.101";//"localhost"; //1. SERVER_IP: the IP address of the computer running the server application
	private static final int TEAM_NUMBER = 6;	// 2. TEAM_NUMBER: your project team number

		
	public static void main(String[] args) 
	{
		odometer = new Odometer();
		odometer.start();
		
		odometryDisplay = new OdometryDisplay();
		odometryDisplay.start();
		
		nav = new Navigator();
		nav.start();
		
		parseParameters();
		readyPosition();
		
		Player player = new Player();
		player.startPlaying();		
	}
	
	private static void parseParameters() //start wificonnection class, establish connection and set variables
	{
		
		WifiConnection conn = null;
		
		try {
			conn = new WifiConnection(SERVER_IP, TEAM_NUMBER);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		if (conn != null){
			
			HashMap<String,Integer> t = conn.StartData;
			if (t == null)
				System.out.println("Failed to read transmission");
			else {
				System.out.println("Transmission read");
				
				// initialize variables
				startingCorner = t.get("SC");
				role = t.get("Role");
				goalWidth = t.get("w1");
				defenderLine = t.get("d1");
				forwardLine = t.get("d2");
				lowerLeftX = t.get("ll-x");
				lowerLeftY = t.get("ll-y");
				upperRightX = t.get("ur-x");
				upperRightY = t.get("ur-y");
				ballColour = t.get("BC");
				
			}
			
		} else {
			System.out.println("Transmission failed");
		}
				
	}
	private static void readyPosition() //start localization
	{
		USLocalizer usL = new USLocalizer();
		usL.doLocalization();
		LineSquarer lineSquarer = new LineSquarer();
		lineSquarer.squareWithLines();
		//once you get here, the robot will be at what it thinks is 0,0 and heading in 0. If the starting corner was 1, this would be fine.
		
		switch(startingCorner)
		{
		case 1: 
			odometer.setX(0);
			odometer.setY(0);
			odometer.setTheta(0);
			System.out.println("Case 1");
			break;
		case 2:
			odometer.setX(300);
			odometer.setY(0);
			odometer.setTheta(0.5*Math.PI);
			System.out.println("Case 2");
			break;
		case 3:
			odometer.setX(300);
			odometer.setY(300);
			odometer.setTheta(Math.PI);
			System.out.println("Case 3");
			break;
		case 4:
			odometer.setX(0);
			odometer.setY(300);
			odometer.setTheta(1.5*Math.PI);
			System.out.println("Case 4");
			break;
		}
	}

}
