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
		//Baby Driver
		//main.pov180.whenPressed(new DriveGoToPeg());
		main.pov90.whileHeld(new DriveAddSensitiveRight());
		main.pov270.whileHeld(new DriveAddSensitiveLeft());


		main.start.whenPressed(new ExpDrive());
		
		main.a.whileHeld(new CubeIntakeRollersIn());
		main.b.whileHeld(new CubeIntakeRollersOut());
		
		main.lb.toggleWhenPressed(new SwitchLeftSolenoid());
		main.rb.toggleWhenPressed(new SwitchRightSolenoid());
		main.y.toggleWhenPressed(new SwitchSolenoids());
		
		main.pov0.whileHeld(new ElevatorUp());
		main.pov180.whileHeld(new ElevatorDown());
		
		//Setting intakePower = new Setting("intakePower", .5);
//		main.a.whileHeld(new MotorsOn(new int[] {4,5}, intakePower.getValueFetched(), "Side Intakes"));//4,5
//		//main.a.toggleWhenPressed(new MotorsOn(new int[] {6}, .5, "Bottom Roller"));
//		
//		main.b.whileHeld(new MotorsOn(new int[] {4,5}, -(intakePower.getValueFetched()), "inv Side Intakes"));//toggleWhenPressed
		
//		main.x.toggleWhenPressed(new MotorsOn(new int[] {4,5}, 0, "off Side Intakes"));
//		
//		main.a.whileHeld(new MotorsOn(new int[] {4}, intakePower.getValueFetched(), "Side Intakes"));
//		main.a.whileHeld(new MotorsOn(new int[] {5}, -(intakePower.getValueFetched()), "Side Intakes"));
//		
//		main.b.whileHeld(new MotorsOn(new int[] {4}, -(intakePower.getValueFetched()), "inv Side Intakes"));
//		main.b.whileHeld(new MotorsOn(new int[] {5}, intakePower.getValueFetched(), "inv Side Intakes"));
		
		
		//main.b.toggleWhenPressed(new MotorsOn(new int[] {6}, -.5, "inv Bottom Roller"));
//		main.x.whileHeld(new DriveAddSensitiveLeft());
//		main.y.whileHeld(new DriveAddSensitiveRight());
		

//		main.start.whenPressed(new DriveChangeToGearDirection());
//		main.back.whenPressed(new DriveChangeToBallDirection());
		



		
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
