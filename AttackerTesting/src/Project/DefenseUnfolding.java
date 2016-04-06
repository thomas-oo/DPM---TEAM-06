package Project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class DefenseUnfolding {
	//left motor = large wing
	static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	//right motor = short wing
	static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	final static int speed = 200;
	
	// two possible states to be in, defense or offense, this can be used in the final project
	enum State {DEF,OF};
	public static void main (String[]args)
	{
		State state = State.OF;

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		switch(state)
		{
		//unfolding of the wings
		case DEF:
			//adjust the angle depending on what you need
			leftMotor.rotate(90,true);
			rightMotor.rotate(90,true);

			leftMotor.forward();
			rightMotor.forward();
			break;
		//folding of the wings
		case OF:
			//adjust the angle depending on what you need
			leftMotor.rotate(270,true);
			rightMotor.rotate(-90,true);

			leftMotor.forward();
			rightMotor.forward();
			break;
		}

	}

}
