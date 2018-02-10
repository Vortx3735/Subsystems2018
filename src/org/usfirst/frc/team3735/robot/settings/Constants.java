package org.usfirst.frc.team3735.robot.settings;

public class Constants {
	
	public class Drive{
		
		public static final double InchesPerRotation = 12.94;	//in inches
		
		//exp drive
		public static final double moveReactivity = 1;	//(0,1] (least reactive, most reactive]
		public static final double turnReactivity = 1;	//(0,1] (least reactive, most reactive]	
		public static final double scaledMaxMove = 1;
		public static final double scaledMaxTurn = .6;	//(0,1] directly to the arcadedrive turn value
		//these retain the range but shift more of the action towards lower values as the exponent is raised higher
		//graph y = x * x^(p-1) {-1 < x < 1} for visualization
		public static final double moveExponent = 3;		//[1,inf) 1 is linear, 2 is squared (normal), etc.
		public static final double turnExponent = 3;		//[1,inf) 

		//for turning slowly with lb and rb
		public static final double lowSensitivityLeftTurn = -.2;
		public static final double lowSensitivityRightTurn = .2;
		
		public static final boolean isUsingLeftEncoders = true;
		public static final boolean isUsingRightEncoders = true;
		
		public static final double MAX_SPEED = 100; //in inches per second

	}
	
	public class CubeIntake{
		public static final double cubeIntakeSpeed = .5;
	}
	
	public class Elevator{
		public static final double elevatorSpeed = 0.5;
		public static final double carriageSpeed = 1.0;
		
		public static final double elevatorMultiplier = 0.2;
		public static final double correctionMultiplier = 0.01;
		
		public static final double ticksPerInch =500; 
		
		public static final double dPLeft = 1/ticksPerInch;
		public static final double dILeft = 0.0;
		public static final double dDLeft = 0.0;
		public static final double dFLeft = 0.0;
		public static final int iZoneLeft = 2;
		
		public static final double dPRight = 1/ticksPerInch;
		public static final double dIRight = 0.0;
		public static final double dDRight = 0.0;
		public static final double dFRight = 0.0;
		public static final int iZoneRight = 2;
	}
	
	public class Climber{
		public static final double initialSpeed = 0.5;
		public static final double tensionSpeed = 1.0;		
	}

}