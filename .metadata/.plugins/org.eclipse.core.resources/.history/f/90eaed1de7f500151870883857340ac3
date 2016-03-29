package Robot;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class BallGrab { 
	
	//color being passed into the class (1 for blue, 2 for red)
	// int color;
	
	//the wheels of the robot
	EV3LargeRegulatedMotor mainLeftMotor; //master
	EV3LargeRegulatedMotor mainRightMotor; //master 
	final static int normalSpeed = 200;
	final static int detectionSpeed = 50;
	
	//for the color sensor to be used
	EV3ColorSensor colorSensor;
	float[] colorData;
	SensorMode colorValue;
	
	//for the UltrasonicSensor
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usSampleProvider;
	public double usDistance;
	private float[] usData;
	
	//V1.0  for BallGrab_V1.0
	static final EV3MediumRegulatedMotor grabMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	final static int speed =  4000;
	
	//V2.0  for BallGrab_V2.0
	static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); //slave
	static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")); // slave
	final static int loweringSpeed = 100;
	final static int forwardSpeed = 600;
	final static int holdingSpeed = 50;
	final static int acceleration = 6000;
	
	public BallGrab()//int color)
	{
		//this.color = color;	
		this.mainLeftMotor = Main.leftMotor;
		this.mainRightMotor = Main.rightMotor;

		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usSampleProvider = usSensor.getMode("Distance");
		this.usData = new float[((BaseSensor) usSampleProvider).sampleSize()];
	}
	
	public void run ()
	{
		//when the robot find a ball
		if (ballDetected())
		{
			//findColor();
			
			mainMotorSetSpeeds(normalSpeed,normalSpeed);
			mainLeftMotor.rotate(-30,true); //CHANGE VALUE SO THAT IT ROTATES TO MIDWAY ACCROSS THE BALL
			mainRightMotor.rotate(-30,false);
			
			//turn and move forward to grab the ball
			mainLeftMotor.rotate(-90,true);
			mainRightMotor.rotate(90,false);
			mainLeftMotor.rotate(360, true);
			mainRightMotor.rotate(360,false);
			
			//V2.0  for BallGrab_V2.0
			motorSetSpeeds(loweringSpeed,loweringSpeed);
			leftMotor.rotate(110,true);
			rightMotor.rotate(110,false);

			//V1.0  for BallGrab_V1.0
			grabMotor.setSpeed(speed);
			grabMotor.rotate(50);
			grabMotor.rotate(-45);
			grabMotor.rotate(45);
			
			//hold the ball at a rather vertical angle
			motorSetSpeeds(holdingSpeed,holdingSpeed);
			leftMotor.rotate(-11,true);
			rightMotor.rotate(-110,false);
			
		}
		
		//use navigation class to navigate to the designated area, then shoot 
		
		//V2.0  for BallGrab_V2.0
/*		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);*/
		grabMotor.rotate(-10);
	}
	
	//returns true when the Robot detects a ball
	public boolean ballDetected()
	{
		usSampleProvider.fetchSample(usData, 0);
		usDistance = (double)(usData[0]*100.0);
		
		//while the robot isn't seeing an object
		while (usDistance>100)
		{
			mainMotorSetSpeeds(normalSpeed,normalSpeed);
			mainLeftMotor.forward();
			mainRightMotor.forward();
		}
		mainLeftMotor.stop();
		mainRightMotor.stop();
		
		return true;
	}
	
	//finding out what color the ball detected is
	/*
	private void findColor()
	{
		int redColorCount = 0;
		int blueColorCount = 0;
		
		BallColorDetector detector = new BallColorDetector(colorSensor, colorData, colorValue);
		detector.start();
		
		usSampleProvider.fetchSample(usData, 0);
		usDistance = (double)(usData[0]*100.0);
		
		while (usDistance>100)
		{				
			mainLeftMotor.setSpeed(detectionSpeed); //slow down
			mainRightMotor.setSpeed(detectionSpeed); //slow dow
			mainLeftMotor.forward();
			mainRightMotor.forward();
			
			if (detector.getColor() == 1)
			{
				redColorCount ++;
			}
			
			if (detector.getColor() == 2)
			{
				blueColorCount ++;
			}
		}
		
		mainLeftMotor.stop();
		mainRightMotor.stop();
		
		if (redColorCount > blueColorCount)
		{
			color = 2; //the color detected was red
		}
		
		if (blueColorCount > redColorCount)
		{
			color = 1; // the color detected was blue
		}

	}
	*/
	
	public void motorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}
	
	public void mainMotorSetSpeeds(int leftSpeed, int rightSpeed) //sets motor speeds
	{
		mainLeftMotor.setSpeed(leftSpeed);
		mainRightMotor.setSpeed(rightSpeed);
	}


}
