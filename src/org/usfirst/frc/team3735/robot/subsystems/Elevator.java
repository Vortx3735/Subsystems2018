package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.commands.elevator.BlankPID;
import org.usfirst.frc.team3735.robot.commands.elevator.ElevatorMove;
import org.usfirst.frc.team3735.robot.settings.Constants;
import org.usfirst.frc.team3735.robot.settings.RobotMap;
import org.usfirst.frc.team3735.robot.util.VorTxTalon;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	VorTxTalon elevatorLeft;
	VorTxTalon elevatorRight;
	
	WPI_TalonSRX carriageLeft;
	WPI_TalonSRX carriageRight;
	
	private Setting speed;
	private Setting carriageSpeed;
	
	private Setting elevatorMultiplier;
	private Setting correctionMultiplier;
	
//	private Setting dPLeft;
//	private Setting dILeft;
//	private Setting dDLeft;
//	private Setting dFLeft;
	private PIDController leftPID;
	private Setting iZoneLeft;
	
//	private Setting dPRight;
//	private Setting dIRight;
//	private Setting dDRight;
//	private Setting dFRight;
	
	private PIDController rightPID;
	private Setting iZoneRight;
	
	public Elevator(){
		elevatorLeft = new VorTxTalon(RobotMap.Elevator.elevatorLeft, "Elevator Left");
		elevatorRight = new VorTxTalon(RobotMap.Elevator.elevatorRight, "Elevator Right");
		
		carriageLeft = new WPI_TalonSRX(RobotMap.Elevator.carriageLeft);
		carriageRight = new WPI_TalonSRX(RobotMap.Elevator.carraigeRight);
		
		speed = new Setting("Elevator Speed", Constants.Elevator.elevatorSpeed);
		carriageSpeed = new Setting("Carriage Speed", Constants.Elevator.carriageSpeed);
		elevatorMultiplier = new Setting("Elevator Move Multiplier", Constants.Elevator.elevatorMultiplier);
		correctionMultiplier = new Setting("Elevator Correct Multiplier", Constants.Elevator.correctionMultiplier);

		SmartDashboard.putData("Left PID", elevatorLeft.getPIDController());
		SmartDashboard.putData("Right PID", elevatorRight.getPIDController());
		
		elevatorLeft.setNeutralMode(NeutralMode.Brake);
		elevatorRight.setNeutralMode(NeutralMode.Brake);
		
		elevatorRight.setInverted(true);
		carriageRight.setInverted(true);
		
		setUpSensors();
		resetEncoderPositions();
		//setupForPositionControl();
		//setUpSlaves();
	}
	
	public void setCarriageLeftCurrent(double speed){
		carriageLeft.set(speed);
	}
	
	public void setCarriageRightCurrent(double speed){
		carriageRight.set(speed);
	}
	
	public void setCarriageCurrent(double speed){
		setCarriageLeftCurrent(speed);
		setCarriageRightCurrent(speed);
	}
	
//	public void setUpSlaves(){
//		motor2.follow(motor1);
//	}
	
	public void setUpSensors(){
		elevatorLeft.setSensorType(FeedbackDevice.QuadEncoder);
		elevatorRight.setSensorType(FeedbackDevice.QuadEncoder);
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
		elevatorLeft.setPIDF(kp, ki, kd, kf);
	}
	
	public void setElevatorRightPIDF(double kp, double ki, double kd, double kf) {
		elevatorRight.setPIDF(kp, ki, kd, kf);
	}

	public void setElevatorLeftPID(double kp, double ki, double kd){
		elevatorLeft.setPID(kp, ki, kd);
	}
	public void setElevatorRightPID(double kp, double ki, double kd){
		elevatorRight.setPID(kp, ki, kd);
	}
//	/
	
	public void setupForPositionControl() {
		//setPIDFSettings(dP,dI,dD,dF);
//		setElevatorLeftPIDF(dPLeft.getValueFetched(), dILeft.getValueFetched(), dDLeft.getValueFetched(), dFLeft.getValueFetched());
//		setElevatorRightPIDF(dPRight.getValueFetched(), dIRight.getValueFetched(), dDRight.getValueFetched(), dFRight.getValueFetched());
		
		elevatorLeft.configAllowableClosedloopError(0, 0, 0);
//		elevatorLeft.config_IntegralZone(0, (int)iZoneLeft.getValue(), 0);	
		
		elevatorRight.configAllowableClosedloopError(0, 0, 0);
//		elevatorRight.config_IntegralZone(0, (int)iZoneRight.getValue(), 0);
		updatePID();
	}

	
	public void resetEncoderPositions(){
		//int absolutePosition = elevatorLeft.getSelectedSensorPosition(0) & 0x0000;
		elevatorLeft.setSelectedSensorPosition(0, 0, 0);
		
		//I think we can just pass in zero with the new stuff like so put we should test it
		
		//absolutePosition = elevatorRight.getSelectedSensorPosition(0) & 0x0000;
		elevatorRight.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void setElevatorMotorsCurrent(double speed){
		setElevatorLeftCurrent(speed);
		setElevatorRightCurrent(speed);
    }
	
	public void setElevatorLeftCurrent(double speed){
		elevatorLeft.set(ControlMode.PercentOutput, speed);
	} 
	public void setElevatorRightCurrent(double speed){
		elevatorRight.set(ControlMode.PercentOutput, speed);
	} 
	
	public void setElevatorLeftPosition(double position){
		elevatorLeft.set(ControlMode.Position, position);	
	}
	
	public void setElevatorRightPosition(double position){
		elevatorRight.set(ControlMode.Position, position);
	}
	
	public void setElevatorPostion(double position){
		setElevatorLeftPosition(position);
		setElevatorRightPosition(position);
	}
//	public void moveElevatorInches(double inches){
//		double ticksToMove = (inches*Constants.Elevator.ticksPerInch);
//		elevatorLeft.set(ControlMode.Position, ticksToMove);
//		elevatorRight.set(ControlMode.Position, ticksToMove);
//	}

	
    public double getSpeedSmartDashboard(){
    	return speed.getValue();
    }
    public double getCarriageSpeedSmartDashboard(){
    	return carriageSpeed.getValue();
    }
    
    public double getMultiplierSmartDashboard(){
    	return elevatorMultiplier.getValue();
    }
    public double getCorrectionMultiplierSmartDashboard(){
    	return correctionMultiplier.getValue();
    }
    /*
     * 2 = 
     */
    public void updatePID() {
    	elevatorLeft.updatePID();
    	elevatorRight.updatePID();
    }
    

    public void initDefaultCommand() {
    	setDefaultCommand(new ElevatorMove());
    }
    
    public void log(){
    	elevatorLeft.putToDashboard();
    	elevatorRight.putToDashboard();
    }
}

