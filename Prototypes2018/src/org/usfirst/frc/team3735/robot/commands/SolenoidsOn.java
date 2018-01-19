package org.usfirst.frc.team3735.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;

import org.usfirst.frc.team3735.robot.Robot;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.CANTalon;

/**
 *
 */
public class SolenoidsOn extends Command {
	ArrayList<Solenoid> solenoids;

	public SolenoidsOn(int[] ports) {
		solenoids = new ArrayList<Solenoid>();
		for(int i = 0; i < ports.length; i++) {
			int port = ports[i];
			if(port < 1 || port > 4) {
				System.out.println("This Solenoid port value is invalid: " + ports);
				continue;
			}
			solenoids.add(new Solenoid(port));
		}

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		for(Solenoid sol : solenoids) {
			sol.set(true);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		for(Solenoid sol : solenoids) {
			sol.set(false);
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
