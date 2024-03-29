package org.usfirst.frc.team3735.robot.commands.elevator;

import org.usfirst.frc.team3735.robot.Robot;
import org.usfirst.frc.team3735.robot.util.settings.Func;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorSetPosition extends Command {
	Func inches;
    public ElevatorSetPosition(double inches) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this(new Func() {
    		@Override
    		public double getValue() {
    			return inches;
    		}
    	});
    	requires(Robot.elevator);
    	
    	
    }
    
    public ElevatorSetPosition(Func f) {
    	this.inches = f;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.setElevatorPostion(inches.getValue());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.setElevatorMotorsCurrent(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
