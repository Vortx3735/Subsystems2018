package org.usfirst.frc.team3735.robot.commands.drive;

import org.usfirst.frc.team3735.robot.Robot;

import org.usfirst.frc.team3735.robot.subsystems.Navigation;
import org.usfirst.frc.team3735.robot.subsystems.Vision.Target;
import org.usfirst.frc.team3735.robot.util.calc.VortxMath;
import org.usfirst.frc.team3735.robot.util.profiling.Location;
import org.usfirst.frc.team3735.robot.util.settings.Func;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnTo extends Command{
	
	private double finishTime = .5;
	private double timeOnTarget = 0;

	Func getAngle;

	public TurnTo(double angle) {
    	this(new Func(){
			@Override
			public double getValue() {
				return angle;
			}
    	});
    }
	
	
	public TurnTo(Target p) {
    	this(new Func(){
			@Override
			public double getValue() {
				return VortxMath.navLimit(
	    				Robot.navigation.getYaw() + Robot.vision.getRelativeCXAngle(p)
	    		);
			}
    	});
    	requires(Robot.vision);
    }
	
	public TurnTo(Location loc) {
		this(new Func(){
			@Override
			public double getValue() {
				return Robot.navigation.getAngleToLocation(loc);
			}
    	});
    	
    }
	
	public TurnTo(Func fun) {
    	requires(Robot.drive);
    	requires(Robot.navigation);
		getAngle = fun;
	}
	

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.navigation.getController().setSetpoint(getAngle.getValue());
    	Robot.navigation.getController().enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.navigation.getController().setIZone(Navigation.iZone.getValue());
    	Robot.navigation.getController().updateI(Navigation.actingI.getValue());
    	
    	if(Robot.navigation.getController().onTarget()){
    		timeOnTarget += .02;
    	}else{
    		timeOnTarget = 0;
    	}
    }

	// Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return timeOnTarget >= finishTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.navigation.getController().disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }

}
