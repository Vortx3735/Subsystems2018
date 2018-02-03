package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.settings.Constants;
import org.usfirst.frc.team3735.robot.settings.RobotMap;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	WPI_TalonSRX motor1;
	WPI_TalonSRX motor2;
	
	WPI_TalonSRX leftMotor;
	WPI_TalonSRX rightMotor;
	
	private Setting speed;
	private Setting carriageSpeed;
	
	private static double dP = 1.0;
	private static double dI = 0.0;
	private static double dD = 0.0;
	private static double dF = 0.0;
	private static int iZone = 2;
	
	public Elevator(){
		motor1 = new WPI_TalonSRX(RobotMap.Elevator.motor1);
		motor2 = new WPI_TalonSRX(RobotMap.Elevator.motor2);
		
		leftMotor = new WPI_TalonSRX(RobotMap.Elevator.leftMotor);
		rightMotor = new WPI_TalonSRX(RobotMap.Elevator.rightMotor);
		
		speed = new Setting("Elevator Speed", Constants.Elevator.elevatorSpeed);
		carriageSpeed = new Setting("Carriage Speed", Constants.Elevator.carriageSpeed);
		
		setUpSensor();
		setUpSlaves();
	}
	
	public void setLeftMotorCurrent(double speed){
		leftMotor.set(speed);
	}
	
	public void setRightMotorCurrent(double speed){
		rightMotor.set(speed);
	}
	
	public void setUpSlaves(){
		motor2.follow(motor1);
	}
	
	public void setUpSensor(){
		int absolutePosition = motor1.getSelectedSensorPosition(0) & 0xFFF;

		//l1.reverseOutput(false); <--- setinverted does this instead

		motor1.setSelectedSensorPosition(absolutePosition, 0, 0);
		motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		motor1.setSensorPhase(true);
	}
	
	public void setPID(double kp, double ki, double kd){
		motor1.config_kP(0, kp, 0);
		motor1.config_kI(0, ki, 0);
		motor1.config_kD(0, kd, 0);
	}
	
	public void setPIDF(double kp, double ki, double kd, double kf) {
		setPID(kp,ki,kd);
		motor1.config_kF(0, kf, 0);
	}
	
	public void setupForPositionControl() {
		motor1.configAllowableClosedloopError(0, 0, 0);
		setPIDF(dP,dI,dD,dF);
		motor1.config_IntegralZone(0, iZone, 0);		
	}

	
	public void resetEncoderPositions(){
		int absolutePosition = motor1.getSelectedSensorPosition(0) & 0xFFF;
		motor1.setSelectedSensorPosition(absolutePosition, 0, 0);
	}
	
	public void setMotorsCurrent(double speed){
    	motor1.set(ControlMode.Position, speed);
    }
    public double getSpeedSmartDashboard(){
    	return speed.getValueFetched();
    }
    public double getCarriageSpeedSmartDashboard(){
    	return carriageSpeed.getValueFetched();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

