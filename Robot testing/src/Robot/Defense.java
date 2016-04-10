package Robot;

import java.io.IOException;

import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.RegulatedMotor;
public class Defense 
{
	public EV3MediumRegulatedMotor defenseMotor;
	final static int holdSpeed = 0;
	final static int openSpeed = 100;
	final static int holdacc = 2000;
	final static int openacc = 100;
	
	public Defense()
	{
		String name = "slave";
		RemoteRequestEV3 slave = null;
		try {
			slave = new RemoteRequestEV3(BrickFinder.find(name)[0].getIPAddress());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		defenseMotor = new EV3MediumRegulatedMotor(slave.getPort("C"));
		//defenseMotor = slave.createRegulatedMotor("C", 'M');
	}
	
	public void mode(int status) 
	{
		if (status==0) //float
		{
			defenseMotor.setAcceleration(0);
			defenseMotor.setSpeed(0);
			defenseMotor.flt();
			System.out.println("Mode0");
			return;
		}
		if (status==1) //hold
		{
			defenseMotor.setAcceleration(holdacc);
			defenseMotor.setSpeed(holdSpeed);	
			defenseMotor.lock(100);
			System.out.println("Mode1");
			return;
		}
		if (status==2) //open
		{
			System.out.println("Mode2");
			defenseMotor.setAcceleration(openSpeed);
			defenseMotor.setSpeed(openacc);	
			defenseMotor.rotateTo(90);
			defenseMotor.lock(50);
			//defensemotor.setAcceleration(holdacc);
			//defensemotor.setSpeed(holdSpeed);	
			
			return;
		}
		if (status==3) //close
		{
			System.out.println("Mode3");
			defenseMotor.setAcceleration(openSpeed);
			defenseMotor.setSpeed(openacc);	
			defenseMotor.rotateTo(0);
			defenseMotor.lock(50);
			//defensemotor.setAcceleration(holdacc);
			//defensemotor.setSpeed(holdSpeed);	
			
			return;
		}
		if (status==4) //???
		{
			System.out.println("Mode4");
			defenseMotor.lock(50);
			return;
		}
		return;
	}
	
}
