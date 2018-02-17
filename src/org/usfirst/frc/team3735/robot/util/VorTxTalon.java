package org.usfirst.frc.team3735.robot.util;
import org.usfirst.frc.team3735.robot.commands.elevator.BlankPID;
import org.usfirst.frc.team3735.robot.util.settings.Setting;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class VorTxTalon extends WPI_TalonSRX{
	int id;
	String name;
	boolean nameGiven;
	private PIDController PID;
	private Setting iZone;
	
	double P = 0;
	double I = 0;
	double D = 0;
	double F = 0;
	double iZ = 0;
	
	double ticksPerInch = 1;
	
	public VorTxTalon(int id){
		super(id);
		this.id = id;
		PID = new PIDController(P, I, D, new BlankPID(), new BlankPID());
		iZone = new Setting("iZoneLeft", iZ);
	}
	
	public VorTxTalon(int id, String name){
		super(id);
		this.id = id;
		this.name= name;
		nameGiven = true;
		PID = new PIDController(P, I, D, new BlankPID(), new BlankPID());
		iZone = new Setting("iZoneLeft", iZ);
	}

	public void setPID(double kp, double ki, double kd){
		this.config_kP(0, kp, 0);
		this.config_kI(0, ki, 0);
		this.config_kD(0, kd, 0);
		
		this.P = kp;
		this.I = ki;
		this.D = kd;
	}
	
	public void setPIDF(double kp, double ki, double kd, double kf) {
		setPID(kp,ki,kd);
		this.config_kF(0, kf, 0);
		
		this.F = kf;
	}
	
	public void setTicksPerInch(double ticks){
		this.ticksPerInch = ticks;
	}
	
	public void setSensorType(FeedbackDevice device){
		int absolutePosition = this.getSelectedSensorPosition(0) & 0xFFF;
		this.setSelectedSensorPosition(absolutePosition, 0, 0);
		
		this.configSelectedFeedbackSensor(device, 0, 0);

		this.setSensorPhase(true);
		
		//this.configNominalOutputVoltage(0.0, -0.0);
		this.configNominalOutputForward(0, 0);
		this.configNominalOutputReverse(0, 0);
		this.configPeakOutputForward(1, 0);
		this.configPeakOutputReverse(-1, 0);
	}
	
	public void updatePID() {
    	this.setPIDF(PID.getP()/ticksPerInch, PID.getI()/ticksPerInch, PID.getD()/ticksPerInch, PID.getF());
    	this.config_IntegralZone(0,  (int)(iZone.getValue() * ticksPerInch), 0);
    }
	
	@Override
	public void set(ControlMode mode, double value){
		if(mode.equals(ControlMode.Position)){
			updatePID();
			value *= ticksPerInch;
		}
		super.set(mode, value);
	}
	
	public PIDController getPIDController(){
		return PID;
	}
	
	public void putToDashboard(){
		String key;
		if(nameGiven){
			key = name;
		}else{
			key = "Talon " + id;
		}
		SmartDashboard.putNumber((key + " Pos") , this.getSelectedSensorPosition(0));
		SmartDashboard.putNumber((key + " Inches"),this.getSelectedSensorPosition(0)/ticksPerInch);
	}
}
