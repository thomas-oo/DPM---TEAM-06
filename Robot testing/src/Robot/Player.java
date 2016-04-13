package Robot;

public class Player
{
	
	
	// classes used by Player
	private Navigator navigator;
	private BallGrab ballGrabber;

	// role taken by Player
	private enum Role {Attacker, Defender};
	private Role role;
		
	// the corner at which the player starts
	private int startingCorner;
	
	// possible locations the robot may want to travel to
	// Home is the Red region for the Attacker (AttackBase), the green region for the Defender (DefenseBase)
	// Ball Platform is where the ball platform is located
	// Shooting Region is the location from which the player will shoot the ball
	// X1..X4 are the four corners of the grid
	private enum Location {Home, BallPlatform, ShootingRegion, X1, X2, X3, X4, AttackBase, DefenseBase, DefWay3, DefWay4};
	
	// coordinates of the locations on the grid the player may want to travel to
	private final double[] ATTACK_BASE = {5*30, 0.5*Main.forwardLine};
	private final double[] DEFENSE_BASE = {5*30, 270};
	private final double[][] X = {{-0.5,-0.5}, {10.5, -0.5}, {10.5, 10.5}, {-0.5, 10.5}};
	private final double CENTER = 5*30.0;
	private int[] ballPlatform = new int[2];
	
	// constructor -- instantiate objects, initialize variables. Just the usual...
	public Player() {
		
		if (Main.role == 0) this.role = Role.Attacker;
		if (Main.role == 1) this.role = Role.Defender;
		
		navigator = Main.nav;
		
		ballGrabber = new BallGrab();
		
		ballPlatform[0] = Main.upperRightX + 35; //120
		ballPlatform[1] = Main.upperRightY - 8; //165
		
		startingCorner = Main.startingCorner;
		
		
	}
	
	// method that will be called from Main to make the robot start playing
	// after localization is complete
	public void startPlaying() {
		
		// make the player travel to its home base
		travelTo(Location.Home);
		
		// once it has travelled to its home base, start performing 
		// the appropriate tasks -- whether that is attack or defend
		if (role == Role.Defender) defend();
		else if (role == Role.Attacker) attack();
		
	}
	
	// code to expand the defense wall
	private void defend() {
		
		navigator.setPlaying(true);
		travelTo(Location.DefenseBase);
		navigator.turnTo(0);
		//unfold
		Defense def = Main.def;
		def.mode(2);
	}
	
	// code to attack
	private void attack() {
		
		navigator.setPlaying(true);
		
		// travel to the ball platform and grab ball
		travelTo(Location.BallPlatform);
		navigator.turnTo(0);
		ballGrabber.grabBall();
		
		// travel to the shooting region and throw ball
		travelTo(Location.ShootingRegion);
		navigator.turnTo(0.5*Math.PI);
		ballGrabber.throwBall();
	
	}
	
	// code to travel to a certain destination

	private void travelTo(Location destination) 
	{
		System.out.println(destination.name());
		if (destination == Location.Home) {
			
			if (role == Role.Attacker) {
				
				switch(startingCorner) { 
					/*case 1: travelTo(Location.X4); travelTo(Location.AttackBase); break;
					case 2: travelTo(Location.X3); travelTo(Location.AttackBase); break;
					case 3: case 4: travelTo(Location.AttackBase);*/
				case 1: travelTo(Location.AttackBase); break;
				case 2: travelTo(Location.AttackBase); break;
				case 3: //travelTo(Location.X2);
						travelTo(Location.AttackBase); break;
				case 4: //travelTo(Location.X1);
						travelTo(Location.AttackBase); break;
				}
				
			} else if (role == Role.Defender) {
				
				switch(startingCorner) {
					case 1: //travelTo(Location.X4);
							travelTo(Location.DefenseBase); break;
					case 2: //travelTo(Location.X3);
							travelTo(Location.DefenseBase); break;
					case 3: travelTo(Location.DefWay3);
							travelTo(Location.DefenseBase); break;
					case 4: travelTo(Location.DefWay4);
							travelTo(Location.DefenseBase); break;
				}
				
			}
			
		}
			
		else if (destination == Location.BallPlatform)
			navigator.travelTo(ballPlatform[0], ballPlatform[1]);
			
		else if (destination == Location.ShootingRegion)
			navigator.travelTo(CENTER, 7 * 30.0); // we have to account for the case when there is an obstacle in the destination
			// also need to find a way of determining the y coordinate

		else if (destination == Location.AttackBase)
			navigator.travelTo(ATTACK_BASE[0], ATTACK_BASE[1]); 
		
		else if (destination == Location.DefenseBase)
			navigator.travelTo(DEFENSE_BASE[0], DEFENSE_BASE[1]);
		
		else if (destination == Location.X1) 
			navigator.travelTo(X[0][0] * 30.0, X[0][1] * 30.0);

		else if (destination == Location.X2)
			navigator.travelTo(X[1][0] * 30.0, X[1][1] * 30.0);
		
		else if (destination == Location.X3) 
			navigator.travelTo(X[2][0] * 30.0, X[2][1] * 30.0);

		else if (destination == Location.X4)
			navigator.travelTo(X[3][0] * 30.0, X[3][1] * 30.0);
		
		else if (destination == Location.DefWay3)
			navigator.travelTo(240, 240);
		
		else if (destination == Location.DefWay4)
			navigator.travelTo(60, 240);

		
		// return from method only after navigation is complete
		while (navigator.isNavigating())
		{
			System.out.println("destX: " + navigator.destDistance[0] + "destY: " + navigator.destDistance[1]);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
	} 

}