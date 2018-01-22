package org.usfirst.frc.team3735.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;

import org.usfirst.frc.team3735.robot.Robot;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 *
 */
public class MotorsOn extends Command {
	ArrayList<WPI_TalonSRX> motors;
	private Setting speed;
	public MotorsOn(int[] ports, double spd) {
		this(ports, spd, null);
	}

	public MotorsOn(int[] ports, double spd, String sdbname) {
		motors = new ArrayList<WPI_TalonSRX>();
		for(int i = 0; i < ports.length; i++) {
			int port = Math.abs(ports[i]);
			if(port < 1 || port > 16) {
				System.out.println("This port value is invalid: " + ports[i]);
				continue;
			}
			motors.add(new WPI_TalonSRX(port));
			motors.get(motors.size()-1).setInverted(((int)Math.signum(port) == 1) ? false : true);
		}
		if(sdbname == null) {
			speed = new Setting("", spd, false);
		}else {
			speed = new Setting(sdbname, spd);
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		speed.fetch();
		for(WPI_TalonSRX mtr : motors) {
			mtr.set(speed.getValue());
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		for(WPI_TalonSRX mtr : motors) {
			mtr.set(0);
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
