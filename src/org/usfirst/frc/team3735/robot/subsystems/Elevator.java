package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.commands.elevator.ElevatorMove;
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
	WPI_TalonSRX elevatorLeft;
	WPI_TalonSRX elevatorRight;
	
	WPI_TalonSRX carriageLeft;
	WPI_TalonSRX carraigeRight;
	
	private Setting speed;
	private Setting carriageSpeed;
	
	private Setting elevatorMultiplier;
	private Setting correctionMultiplier;
	
	private Setting dPLeft;
	private Setting dILeft;
	private Setting dDLeft;
	private Setting dFLeft;
	private Setting iZoneLeft;
	
	private Setting dPRight;
	private Setting dIRight;
	private Setting dDRight;
	private Setting dFRight;
	private Setting iZoneRight;
	
	public Elevator(){
		elevatorLeft = new WPI_TalonSRX(RobotMap.Elevator.elevatorLeft);
		elevatorRight = new WPI_TalonSRX(RobotMap.Elevator.elevatorRight);
		
		carriageLeft = new WPI_TalonSRX(RobotMap.Elevator.carriageLeft);
		carraigeRight = new WPI_TalonSRX(RobotMap.Elevator.carraigeRight);
		
		speed = new Setting("Elevator Speed", Constants.Elevator.elevatorSpeed);
		carriageSpeed = new Setting("Carriage Speed", Constants.Elevator.carriageSpeed);
		elevatorMultiplier = new Setting("Elevator Move Multiplier", Constants.Elevator.elevatorMultiplier);
		correctionMultiplier = new Setting("Elevator Correcnt Multiplier", Constants.Elevator.correctionMultiplier);
		
		dPLeft = new Setting("dPLeft", Constants.Elevator.dPLeft);
		dILeft = new Setting("dILeft", Constants.Elevator.dILeft);
		dDLeft = new Setting("dDLeft", Constants.Elevator.dDLeft);
		dFLeft = new Setting("dFLeft", Constants.Elevator.dFLeft);
		iZoneLeft = new Setting("iZoneLeft", Constants.Elevator.iZoneLeft);
		
		dPRight = new Setting("dPRight", Constants.Elevator.dPRight);
		dIRight = new Setting("dIRight", Constants.Elevator.dIRight);
		dDRight = new Setting("dDRight", Constants.Elevator.dDRight);
		dFRight = new Setting("dFRight", Constants.Elevator.dFRight);
		iZoneRight = new Setting("iZoneRight", Constants.Elevator.iZoneRight);
		
		setUpSensors();
		setupForPositionControl();
		//setUpSlaves();
	}
	
	public void setCarriageLeftCurrent(double speed){
		carriageLeft.set(speed);
	}
	
	public void setCarriageRightCurrent(double speed){
		carraigeRight.set(speed);
	}
	
//	public void setUpSlaves(){
//		motor2.follow(motor1);
//	}
	
	public void setUpSensors(){
		int absolutePosition = elevatorLeft.getSelectedSensorPosition(0) & 0xFFF;

		//l1.reverseOutput(false); <--- setinverted does this instead

		elevatorLeft.setSelectedSensorPosition(absolutePosition, 0, 0);
		elevatorLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorLeft.setSensorPhase(true);
		
		absolutePosition = elevatorRight.getSelectedSensorPosition(0) & 0xFFF;

		elevatorRight.setSelectedSensorPosition(absolutePosition, 0, 0);
		elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorRight.setSensorPhase(true);
	}
	
	public void setPIDSettings(double kp, double ki, double kd){
		setElevatorLeftPID(kp, ki, kd);
		setElevatorRightPID(kp, ki, kd);
	}
	
	public void setPIDFSettings(double kp, double ki, double kd, double kf){
		setElevatorLeftPIDF(kp, ki, kd, kf);
		setElevatorRightPIDF(kp, ki, kd, kf);
	}
	
	public void setElevatorLeftPIDF(double kp, double ki, double kd, double kf) {
		setElevatorLeftPID(kp,ki,kd);
		elevatorLeft.config_kF(0, kf, 0);
	}
	
	public void setElevatorRightPIDF(double kp, double ki, double kd, double kf) {
		setElevatorRightPID(kp,ki,kd);
		elevatorRight.config_kF(0, kf, 0);
	}

	public void setElevatorLeftPID(double kp, double ki, double kd){
		elevatorLeft.config_kP(0, kp, 0);
		elevatorLeft.config_kI(0, ki, 0);
		elevatorLeft.config_kD(0, kd, 0);
	}
	public void setElevatorRightPID(double kp, double ki, double kd){
		elevatorRight.config_kP(0, kp, 0);
		elevatorRight.config_kI(0, ki, 0);
		elevatorRight.config_kD(0, kd, 0);
	}
//	/
	
	public void setupForPositionControl() {
		//setPIDFSettings(dP,dI,dD,dF);
		setElevatorLeftPIDF(dPLeft.getValueFetched(), dILeft.getValueFetched(), dDLeft.getValueFetched(), dFLeft.getValueFetched());
		setElevatorRightPIDF(dPRight.getValueFetched(), dIRight.getValueFetched(), dDRight.getValueFetched(), dFRight.getValueFetched());
		
		elevatorLeft.configAllowableClosedloopError(0, 0, 0);
		elevatorLeft.config_IntegralZone(0, (int)iZoneLeft.getValueFetched(), 0);	
		
		elevatorRight.configAllowableClosedloopError(0, 0, 0);
		elevatorRight.config_IntegralZone(0, (int)iZoneRight.getValueFetched(), 0);
	}

	
	public void resetEncoderPositions(){
		int absolutePosition = elevatorLeft.getSelectedSensorPosition(0) & 0xFFF;
		elevatorLeft.setSelectedSensorPosition(absolutePosition, 0, 0);
		
		absolutePosition = elevatorRight.getSelectedSensorPosition(0) & 0xFFF;
		elevatorRight.setSelectedSensorPosition(absolutePosition, 0, 0);
	}
	
	public void setElevatorMotorsCurrent(double speed){
		elevatorLeft.set(ControlMode.Position, speed);
		elevatorRight.set(ControlMode.Position, speed);
    }
	
	public void setElevatorLeftCurrent(double speed){
		elevatorLeft.set(ControlMode.Position, speed);
	} 
	public void setElevatorRightCurrent(double speed){
		elevatorRight.set(ControlMode.Position, speed);
	} 
	
    public double getSpeedSmartDashboard(){
    	return speed.getValueFetched();
    }
    public double getCarriageSpeedSmartDashboard(){
    	return carriageSpeed.getValueFetched();
    }
    public double getMultiplierSmartDashboard(){
    	return elevatorMultiplier.getValueFetched();
    }
    public double getCorrectionMultiplierSmartDashboard(){
    	return correctionMultiplier.getValueFetched();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ElevatorMove());
    }
}

