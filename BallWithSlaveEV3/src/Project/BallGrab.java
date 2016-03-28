package Project;

import java.io.IOException;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.remote.ev3.RemoteEV3;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class BallGrab { 
	
	//V1.0  for BallGrab_V1.0
	//static final EV3MediumRegulatedMotor grabMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	final static int speed =  4000;
	
	//V2.0  for BallGrab_V2.0
	//static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	//static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	static RegulatedMotor grabMotor;
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	final static int loweringSpeed = 100;
	final static int forwardSpeed = 600;
	final static int holdingSpeed = 50;
	final static int acceleration = 6000;
	
	public static void main (String[]args)
	{
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
		
		Port usPort = slave.getPort("S1");
		Port colorPort = slave.getPort("S2");
		
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");
		float[] usData = new float[usValue.sampleSize()];
		
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		SampleProvider colorValue = colorSensor.getMode("Red");
		float[] colorData = new float[colorValue.sampleSize()];
		
		
		//V2.0  for BallGrab_V2.0
		leftMotor.setSpeed(loweringSpeed);
		rightMotor.setSpeed(loweringSpeed);
		leftMotor.rotate(110,true);
		rightMotor.rotate(110,false);
		/*leftMotor.lock(100);
		rightMotor.lock(100);*/

		//V1.0  for BallGrab_V1.0
		grabMotor.setSpeed(speed);
		grabMotor.rotate(50);
		grabMotor.rotate(-45);
		grabMotor.rotate(45);
		
		//V2.0  for BallGrab_V2.0
/*		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);*/
		leftMotor.setSpeed(holdingSpeed);
		rightMotor.setSpeed(holdingSpeed);
		leftMotor.rotate(-90,true);
		rightMotor.rotate(-90,false);
		/*leftMotor.lock(100);
		rightMotor.lock(100);*/
		grabMotor.rotate(-10);
		
		((EV3UltrasonicSensor) usSensor).disable();
		colorSensor.close();
	}


}
