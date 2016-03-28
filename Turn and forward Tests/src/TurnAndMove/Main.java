/*
 * HOW TO CALIBRATE:
 * HEAD ROBOT AT 0 (POSITIVE X AXIS)
 * RUN, IT WILL MOVE TO (60,0)
 * IT WILL THEN TURN TO 0 DEGREES
 * THEN IT WILL TURN 360 DEGREES CCW
 * 
 * Calibrating track and wheel radius:
 * Linear movement: depends only on wheel radius
 * Turning: depends on both wheel radius and track.
 * 
 * Thus calibrate linear movement THEN calibrate turning.
 * Both should be linearly related to the wheel radius and track.
 */

package TurnAndMove;

import TurnAndMove.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Main 
{
	static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	static Odometer odo;
	static Navigator nav;
	static OdometryDisplay odoDisplay;
	final static TextLCD t = LocalEV3.get().getTextLCD();
	
	static double ROTATION_SPEED = 200;
	static double WHEEL_RADIUS = 2.03;
	static double TRACK = 17.45;
	
	public static void main(String[] args) 
	{
		odo = new Odometer();
		odoDisplay = new OdometryDisplay(odo,t);
		odoDisplay.start();
		
		//travel 30 cm forward
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, 30), true);
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, 30), false);
		
		Button.waitForAnyPress();
		turn(3600);
		Button.waitForAnyPress();
		System.exit(0);
	}
	
	private static void turn(double turnAngle)
	{
		leftMotor.setSpeed((int) ROTATION_SPEED);
		rightMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.rotate(convertAngle(-WHEEL_RADIUS, TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, turnAngle), false);
		
		System.out.println("Final angle after 360: " + odo.getTheta());
	}
	
	private static int convertAngle(double radius, double width, double angle) 
	{
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance)
	{
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
