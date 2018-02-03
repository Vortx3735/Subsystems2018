package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.settings.Constants;
import org.usfirst.frc.team3735.robot.settings.RobotMap;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
	
	private static double dP = 1.0;
	private static double dI = 0.0;
	private static double dD = 0.0;
	private static double dF = 0.0;
	private static int iZone = 2;
	
	public Elevator(){
		motor = new WPI_TalonSRX(RobotMap.Elevator.motor);
		speed = new Setting("Elevator Speed", Constants.Elevator.elevatorSpeed);
		
		setUpSensor();
	}
	
	public void setUpSensor(){
		int absolutePosition = motor.getSelectedSensorPosition(0) & 0xFFF;

		//l1.reverseOutput(false); <--- setinverted does this instead

		motor.setSelectedSensorPosition(absolutePosition, 0, 0);
		motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		motor.setSensorPhase(true);
	}
	
	public void setPID(double kp, double ki, double kd){
		motor.config_kP(0, kp, 0);
		motor.config_kI(0, ki, 0);
		motor.config_kD(0, kd, 0);
	}
	
	public void setPIDF(double kp, double ki, double kd, double kf) {
		setPID(kp,ki,kd);
		motor.config_kF(0, kf, 0);
	}
	
	public void setupForPositionControl() {
		motor.configAllowableClosedloopError(0, 0, 0);
		setPIDF(dP,dI,dD,dF);
		motor.config_IntegralZone(0, iZone, 0);		
	}

	
	public void resetEncoderPositions(){
		int absolutePosition = motor.getSelectedSensorPosition(0) & 0xFFF;
		motor.setSelectedSensorPosition(absolutePosition, 0, 0);
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

