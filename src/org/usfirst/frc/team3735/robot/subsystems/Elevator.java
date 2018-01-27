package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.settings.Constants;
import org.usfirst.frc.team3735.robot.settings.RobotMap;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	WPI_TalonSRX motor;
	private Setting speed;
	
	public Elevator(){
		motor = new WPI_TalonSRX(RobotMap.Elevator.motor);
		speed = new Setting("Elevator Speed", Constants.Elevator.elevatorSpeed);
	}
	
	public void setMotorCurrent(double speed){
    	motor.set(speed);
    }
    public double getSpeedSmartDashboard(){
    	return speed.getValueFetched();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

