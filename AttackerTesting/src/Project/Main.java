package Project;


public class Main {
	
	public static BallGrab ballGrabber;
	
	public static void main (String[]args)
	{
		System.out.println("I work");
		ballGrabber = new BallGrab();
		ballGrabber.grabBall();
	}

}
