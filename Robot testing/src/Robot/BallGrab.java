package Robot;

import java.io.IOException;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
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
import lejos.hardware.sensor.EV3TouchSensor;

public class BallGrab 
{
	//V1.0  for BallGrab_V1.0
	static RegulatedMotor grabMotor;
	final static int speed =  200;

	//V2.0  for BallGrab_V2.0
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	/*	static RegulatedMotor grabMotor;
		static RegulatedMotor leftMotor;
		static RegulatedMotor rightMotor;*/
	final static int loweringSpeed = 200;
	final static int forwardSpeed = 900;
	final static int holdingSpeed = 50;
	final static int acceleration = 6000;
	
	public BallGrab()
	{
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
	}
	
	public void grabBall()
	{
		//V2.0  for BallGrab_V2.0
		leftMotor.setSpeed(loweringSpeed);
		rightMotor.setSpeed(loweringSpeed);
		leftMotor.rotate(180,true);
		rightMotor.rotate(180,false);

		//V1.0  for BallGrab_V1.0
		grabMotor.setSpeed(speed);
		grabMotor.rotate(100);
	}

	public void throwBall()
	{
		//V2.0  for BallGrab_V2.0
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.rotate(-150,true);
		rightMotor.rotate(-150,false);


		grabMotor.rotate(-90);
	}
}
