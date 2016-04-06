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
	
	
	//color being passed into the class (1 for red, 2 for blue): targeted color
	int color = 1;
	double wheelRadius = 2.1;
	
	//the wheels of the robot
	public static final EV3LargeRegulatedMotor mainleftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor mainrightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	EV3LargeRegulatedMotor mainLeftMotor; //master: PASSED ON
	EV3LargeRegulatedMotor mainRightMotor; //master: PASSED ON
	
	
	final static int normalSpeed = 200;
	final static int detectionSpeed = 20;
	
	//initiating colorSensor
	private static final Port colorPort = LocalEV3.get().getPort("S4");
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	private static SensorMode colorValue = colorSensor.getMode("RGB");
	private static int sampleSize = colorValue.sampleSize();
	private static float[] colorData = new float[sampleSize];
	
	//in real code, this will be accessed through slave REPLACE THE ABOVE CODE
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
	
	//V1.0  for BallGrab_V1.0 FOR GRABBING
	public static final EV3MediumRegulatedMotor grabMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("S1")); //NOT USED IN ACTUAL CODE
//	static RegulatedMotor grabMotor;//NOT USED IN ACTUAL CODE
	final static int speed =  200;
	
	//V2.0  for BallGrab_V2.0 FOR SHOOTING
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));//NOT USED IN ACTUAL CODE
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));//NOT USED IN ACTUAL CODE
//	static RegulatedMotor leftMotor;//NOT USED IN ACTUAL CODE
//	static RegulatedMotor rightMotor;//NOT USED IN ACTUAL CODE
	final static int loweringSpeed = 200;
	final static int forwardSpeed = 900;
	final static int holdingSpeed = 50;
	final static int acceleration = 6000;
	
	//USED FOR ACCESSING SLAVE
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
	
	//grabbing the right colored ball
	public void grabBall()
	{
		boolean grabbed = false;
		
		//while the robot hasn't grabbed the right colored ball
		while (!grabbed)
		{
			//when the robot find a ball
			if (ballDetection())
			{	
				//if the color found was the color targeted
				if (findColor() == color)
				{				
					System.out.println(findColor());
					
					mainMotorSetSpeeds(normalSpeed,normalSpeed);
					
					//backing up is needed compensate for the offset of the robot when it rotates to grab the ball
					int backingUp =  Main.convertDistance(wheelRadius, 10);
					mainleftMotor.rotate(-backingUp,true); 
					mainrightMotor.rotate(-backingUp,false);
					mainleftMotor.backward();
					mainrightMotor.backward();
					
					//turn and move forward to grab the ball use the localizer class? (stop rotating when you detect two lines): USE LIGHTLOCALIZER CLASS
					mainleftMotor.rotate(360,true);
					mainrightMotor.rotate(-360,false);
					mainleftMotor.forward();
					mainrightMotor.forward();
					
					//move until the robot is far enough to grab the ball
					int DistFromBall = Main.convertDistance(wheelRadius, 33.5);//needs calibration
					mainMotorSetSpeeds(normalSpeed,normalSpeed);
					mainleftMotor.rotate(DistFromBall, true);
					mainrightMotor.rotate(DistFromBall,false);
					mainleftMotor.forward();
					mainrightMotor.forward();
					
					System.out.println("I'm going to grab the ball now");
					
					// THE FOLLOWING THREE PIECES OF CODES WERE NOT TESTED YET
					
					//V2.0  for BallGrab_V2.0
					motorSetSpeeds(loweringSpeed,loweringSpeed);
					leftMotor.rotate(180,true);
					rightMotor.rotate(180,false);

					//V1.0  for BallGrab_V1.0
					grabMotor.setSpeed(speed);
					grabMotor.rotate(100);
					
					//hold the ball at a rather vertical angle
					motorSetSpeeds(holdingSpeed,holdingSpeed);
					leftMotor.rotate(-110,true);
					rightMotor.rotate(-110,false);
					
					grabbed = true;
				}
				
				//if the robot didn't detect the right colored ball, it will move forward a certain distance so that it
				//doesn't see a ball anymore (hardcoded)
				else
				{
					int MidBall =  Main.convertDistance(wheelRadius, 5);
					mainMotorSetSpeeds(normalSpeed,normalSpeed);
					mainleftMotor.rotate(MidBall,true); 
					mainrightMotor.rotate(MidBall,false);
					mainleftMotor.forward();
					mainrightMotor.forward();
				}
			}
		}
	}
	
	//returns true when the robot detects a ball THIS NEEDS TO BE TESTED
	public void throwBall()
	{
		//V2.0  for BallGrab_V2.0
 		motorSetSpeeds(holdingSpeed,holdingSpeed); //have the arm rotate down
		leftMotor.rotate(110,true);
		rightMotor.rotate(110,false);
		
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		motorSetSpeeds(forwardSpeed,forwardSpeed);
		leftMotor.rotate(-150,true);
		rightMotor.rotate(-150,false);


		grabMotor.rotate(-90); //open the claw
		
	}
	
	//returns true when the Robot detects a ball
	private boolean ballDetection()
	{
		usSampleProvider.fetchSample(usData, 0);
		usDistance = (double)(usData[0]*100.0); //use as first comparator
		System.out.println(initialDistance);
		
		int normalSpeed = 50;
		
		//while the robot isn't seeing an object
		while (usDistance==0||usDistance>8) //adjust the value, how reliable is it?
			//the robot will keep moving forward as long as the distance it detected is 0 (some kind of error)
		{
			usSampleProvider.fetchSample(usData, 0);
			usDistance = (double)(usData[0]*100.0);
			System.out.println(usDistance);
			mainMotorSetSpeeds(normalSpeed,normalSpeed);
			mainleftMotor.forward();
			mainrightMotor.forward();
		}
		
		//have the robot move to the middle of the ball.
		int MidBall =  Main.convertDistance(wheelRadius, 2);
		mainMotorSetSpeeds(normalSpeed,normalSpeed);
		mainleftMotor.rotate(MidBall,true); 
		mainrightMotor.rotate(MidBall,false);
		mainleftMotor.forward();
		mainrightMotor.forward();
		
		mainleftMotor.stop();
		mainrightMotor.stop();
		
		return true; //the robot has detected a ball
	}
	
	//finding out what color the ball detected is, relies on one value for efficiency and for speed purposes.
	private int findColor()
	{
		int colorFound = 0;
		
		BallColorDetector detector = new BallColorDetector(colorSensor, colorData, colorValue);
		detector.start(); //detect the color at that specific instant
		
		colorFound = detector.getColor();
		
		return colorFound;
		
	}
	
	
	public void motorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}
	
	public void mainMotorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		mainleftMotor.setSpeed(leftSpeed);
		mainrightMotor.setSpeed(rightSpeed);
	}


}
