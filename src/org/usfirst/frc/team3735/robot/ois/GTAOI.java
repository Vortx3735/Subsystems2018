package org.usfirst.frc.team3735.robot.ois;

//import org.usfirst.frc.team3735.robot.commands.DriveTurnToAngleHyperbola;
import org.usfirst.frc.team3735.robot.commands.*;
import org.usfirst.frc.team3735.robot.commands.cubeintake.*;
import org.usfirst.frc.team3735.robot.commands.drive.ExpDrive;
import org.usfirst.frc.team3735.robot.commands.drive.simple.DriveAddSensitiveLeft;
import org.usfirst.frc.team3735.robot.commands.drive.simple.DriveAddSensitiveRight;
import org.usfirst.frc.team3735.robot.commands.elevator.*;
import org.usfirst.frc.team3735.robot.util.oi.DriveOI;
import org.usfirst.frc.team3735.robot.util.oi.XboxController;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

public class GTAOI implements DriveOI{

	public XboxController main;
	public XboxController co;

	public GTAOI() {

		main = new XboxController(0);
		co = new XboxController(1);
		main.rb.get();

		int[] motors = {8,9};
		main.a.whileHeld(new MotorsOn(motors,0.5));
		main.b.whileHeld(new MotorsOn(motors,-0.5));
//		main.start.whenPressed(new ExpDrive());
//		
//		main.a.whileHeld(new CubeIntakeRollersIn());
//		main.b.whileHeld(new CubeIntakeRollersOut());
//		
//		main.y.toggleWhenPressed(new SwitchSolenoid());
//		
//		main.pov0.whileHeld(new ElevatorUp());
//		main.pov180.whileHeld(new ElevatorDown());
//		
//		co.pov0.whileHeld(new ElevatorUp());
//		co.pov180.whileHeld(new ElevatorDown());
//		co.pov90.whileHeld(new ElevatorCorrectRight());
//		co.pov270.whileHeld(new ElevatorCorrectLeft());
		//main.x.toggleWhenPressed(new ElevatorMoveInches(1));
		
		
//		main.pov0.toggleWhenPressed(new ElevatorMoveInches(1));
//		main.pov180.toggleWhenPressed(new ElevatorMoveInches(-1));
//		main.pov90.toggleWhenPressed(new MoveRightElevatorInches(1));
//		main.pov270.toggleWhenPressed(new MoveLeftElevatorInches(1));
		
		

//		main.x.whileHeld(new DriveAddSensitiveLeft());
//		main.y.whileHeld(new DriveAddSensitiveRight());


		
		co.start.whenPressed(new InterruptOperations());
		
		
		
	}
	
	
	public double getDriveMove() {
		return (main.getRightTrigger() - main.getLeftTrigger());
		//return main.getLeftY();
	}

	public double getDriveTurn() {
		return main.getLeftX();
		//return main.getRightX();
	}
	
	public double getElevatorMove(){
		return main.getRightY();
	}
	
	public double getElevatorTrim(){
		return main.getRightX();
	}
	
	@Override
	public double getFODMag() {
		//return main.getRightMagnitude();
		return 0;
	}
	
	public double getFODAngle(){
		//return main.getRightAngle();
		return 0;
	}

	
	public boolean isOverriddenByDrive(){
		return Math.abs(getDriveMove()) > .4 || Math.abs(getDriveTurn()) > .4;
	}

	
	public void log() {
//		SmartDashboard.putNumber("right joystick angle", getMainRightAngle());
//		SmartDashboard.putNumber("right joystick magnitude",
//				getMainRightMagnitude());

	}




}
