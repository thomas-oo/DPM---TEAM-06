package Project;

import java.io.IOException;

import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class BallGrab { 
	
	
	//color being passed into the class (1 for red, 2 for blue)
	int color = 1;
	
	//the wheels of the robot
	public static final EV3LargeRegulatedMotor mainleftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor mainrightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	EV3LargeRegulatedMotor mainLeftMotor; //master
	EV3LargeRegulatedMotor mainRightMotor; //master 
	final static int normalSpeed = 200;
	final static int detectionSpeed = 20;
	
	//initiating colorSensor
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private static SensorMode colorValue = colorSensor.getMode("RGB");
	private static int sampleSize = colorValue.sampleSize();
	private static float[] colorData = new float[sampleSize];
	
	//in real code, this will be accessed through slave
/*	EV3ColorSensor colorSensor;
	float[] colorData;
	SensorMode colorValue;
	*/
	//initiating UltrasonicSensor
	private static final Port usPort = LocalEV3.get().getPort("S3");
/*	private EV3UltrasonicSensor usSensor;
	private SampleProvider usSampleProvider;
	public double usDistance;
	private float[] usData;*/
	
	public static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usSampleProvider = usSensor.getMode("Distance");
	public static float[] usData = new float[usSampleProvider.sampleSize()];
	public double usDistance;
	public double initialDistance;
	
	//V1.0  for BallGrab_V1.0
//	public static final EV3MediumRegulatedMotor grabMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("S1"));
//	static RegulatedMotor grabMotor;
	final static int speed =  200;
	
	//V2.0  for BallGrab_V2.0
//	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
//	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
/*	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;*/
	final static int loweringSpeed = 200;
	final static int forwardSpeed = 900;
	final static int holdingSpeed = 50;
	final static int acceleration = 6000;
	
	/*public BallGrab(int color)
	{
		this.color = color;	

		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usSampleProvider = usSensor.getMode("Distance");
		this.usData = new float[((BaseSensor) usSampleProvider).sampleSize()];
		
		//accessing the slave ports
		String name = "slave";
		RemoteRequestEV3 slave = null;
		try {
			slave = new RemoteRequestEV3(BrickFinder.find(name)[0].getIPAddress());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		grabMotor = slave.createRegulatedMotor("B", 'M');
		leftMotor = slave.createRegulatedMotor("A", 'L');
		rightMotor = slave.createRegulatedMotor("D", 'L');
	}*/
	
	public void grabBall()
	{
		System.out.println("in grabBall");
		boolean grabbed = false;
		
		while (!grabbed)
		{
			//when the robot find a ball
			if (ballDetection())
			{
				System.out.println("Found ball");
//				if (findColor() == color)
				boolean testing = true;
				if (testing= true)
				{
					int MidBall =  Main.convertDistance(1.5, 4);
					mainMotorSetSpeeds(normalSpeed,normalSpeed);
					mainleftMotor.rotate(-MidBall,true); 
					mainrightMotor.rotate(-MidBall,false);
					mainleftMotor.backward();
					mainrightMotor.backward();
					
					//turn and move forward to grab the ball use the localizer class? (stop rotating when you detect two lines)
					mainleftMotor.rotate(360,true);
					mainrightMotor.rotate(-360,false);
					
					//right now it's rotating a certain number of turns, not always reliable.
					int DistFromBall = Main.convertDistance(1.5, 33.5);
					mainleftMotor.rotate(DistFromBall, true);
					mainrightMotor.rotate(DistFromBall,false);
					
					System.out.println("I'm going to grab the ball now");
					
/*					//V2.0  for BallGrab_V2.0
					motorSetSpeeds(loweringSpeed,loweringSpeed);
					leftMotor.rotate(180,true);
					rightMotor.rotate(180,false);

					//V1.0  for BallGrab_V1.0
					grabMotor.setSpeed(speed);
					grabMotor.rotate(100);
					
					//hold the ball at a rather vertical angle
					motorSetSpeeds(holdingSpeed,holdingSpeed);
					leftMotor.rotate(-110,true);
					rightMotor.rotate(-110,false);*/
					
					grabbed = true;
				}
				else
					grabbed = false;
			}
		}
	}
	
	public void throwBall()
	{
/*		//V2.0  for BallGrab_V2.0
 		motorSetSpeeds(holdingSpeed,holdingSpeed); //have the arm rotate down
		leftMotor.rotate(110,true);
		rightMotor.rotate(110,false);
		
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		motorSetSpeeds(forwardSpeed,forwardSpeed)
		leftMotor.rotate(-150,true);
		rightMotor.rotate(-150,false);


		grabMotor.rotate(-90); //open the claw
		*/
	}
	
	//returns true when the Robot detects a ball
	private boolean ballDetection()
	{
		usSampleProvider.fetchSample(usData, 0);
//		initialDistance = (double)(usData[0]*100.0); //the first distance that it detects (should be away from the platform)
		usDistance = (double)(usData[0]*100.0); //use as first comparator
		System.out.println(initialDistance);

		System.out.println("got my data");
		
		int normalSpeed = 50;
		
		//while the robot isn't seeing an object
		while (usDistance==0||usDistance>7) //the robot will keep moving forward as long as the distance it detected is 0 (some kind of error)
		{
			usSampleProvider.fetchSample(usData, 0);
			usDistance = (double)(usData[0]*100.0);
			System.out.println(usDistance);
			mainMotorSetSpeeds(normalSpeed,normalSpeed);
			mainleftMotor.forward();
			mainrightMotor.forward();
		}
		mainleftMotor.stop();
		mainrightMotor.stop();
		
		return true; 
	}
	
	//finding out what color the ball detected is
	
	private int findColor()
	{
		int redColorCount = 0;
		int blueColorCount = 0;
		int colorFound = 0;
		
		BallColorDetector detector = new BallColorDetector(colorSensor, colorData, colorValue);
		detector.start(); //detect the color at that specific instant
		
		usSampleProvider.fetchSample(usData, 0);
//		initialDistance = (double)(usData[0]*100.0);
		usDistance = (double)(usData[0]*100.0);
		
		while (usDistance!=0 && usDistance<4) //as soon as the ultrasonicSensor detects the next distance to be a large value, it will get out of this loop
		//somehow, this doesn't work, it doesn't go out of the loop even if distance >4
		{			
			usSampleProvider.fetchSample(usData, 0);
			usDistance = (double)(usData[0]*100.0);
			
			System.out.println("usDistance" + usDistance);
			
			mainMotorSetSpeeds(detectionSpeed, detectionSpeed); //slow down
 
			mainleftMotor.forward();
			mainrightMotor.forward();
			
			System.out.println("detector color : " + detector.getColor());
			
			if (detector.getColor() == 1)
			{
				redColorCount ++;
			}
			
			if (detector.getColor() == 2)
			{
				blueColorCount ++;
			}
		}
		
		System.out.println("redColorCount" + redColorCount);
		System.out.println("blueColorCount" + blueColorCount);
		
		mainleftMotor.stop();
		mainrightMotor.stop();
		
		if (redColorCount > blueColorCount)
		{
			colorFound = 1; //the color detected was red
		}
		
		if (blueColorCount > redColorCount)
		{
			colorFound = 2; // the color detected was blue
		}
		
		System.out.println(colorFound);
		return colorFound;
	}
	
	
	/*public void motorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}*/
	
	public void mainMotorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		mainleftMotor.setSpeed(leftSpeed);
		mainrightMotor.setSpeed(rightSpeed);
	}


}
