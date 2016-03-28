package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team3467.robot.OI;

public class PIDRate extends CommandBase {

	/*private PIDController AnglePIDController;
	//private static final double Angle_KP = 0.0;
	//private static final double Angle_KI = 0.0;
	//private static final double Angle_KD = 0.0;
	*/
	
	private  PIDF_CANTalon leftpidf_drive;
	private PIDF_CANTalon rightpidf_drive;
	double angle;
	int mode = 0;
	
	private static final double TOLERANCE = 0.0;
	private static final double KP = 0.0;
	private static final double KI = 0.0;
	private static final double KD = 0.0;
	
	public PIDRate(double angle, int mode) {
		requires(driveBase);
		buildControllers();
		
		this.angle = angle;
		this.mode = mode;
	}
	
	public void buildControllers() {
		leftpidf_drive = new PIDF_CANTalon("Drive Left", driveBase.getLeftTalon(), TOLERANCE, false, false);
		leftpidf_drive.setPID(KP, KI, KD);
		
		rightpidf_drive = new PIDF_CANTalon("Drive Right", driveBase.getLeftTalon(), TOLERANCE, false, false);
		rightpidf_drive.setPID(KP, KI, KD);
	}
	
	//Enable PID in the drive base
	public void enablePID() {
		rightpidf_drive.enable();
		leftpidf_drive.enable();
		
		rightpidf_drive.reset();
		leftpidf_drive.reset();
	}
	
	//Different ways of turning that we need
	public void setStartPID() {	
		switch (mode) {
		case 1:
				leftpidf_drive.setSetpoint(angle * 13.75);
			break;
		case 2:
				rightpidf_drive.setSetpoint(angle * -13.75);
			break;
		default:
				leftpidf_drive.setSetpoint( (angle * 13.75) / 2 );
				rightpidf_drive.setSetpoint( (angle * -13.75) / 2 );
			break;
		}
	}
	
	//Stop running PID in the drive base
	public void stopPID() {
		leftpidf_drive.reset();
		rightpidf_drive.reset();
		
		leftpidf_drive.disable();
		rightpidf_drive.disable();
	}
	
	/*
	public void BuildAnglePIDController() {
		AnglePIDController = new PIDController(Angle_KP, Angle_KI, Angle_KD);
		
		new PIDSource() {
            PIDSourceType m_sourceType = PIDSourceType.kDisplacement;
            
            public double pidGet() {
            	return ahrs.getGyroYaw();
            }
            
            public double pidSet() {
            	return ahrs.setGyroYaw();
            }
            
		}}
	*/
	
	//Sets everything up so that the drive base is ready to run
	protected void initialize() {
		driveBase.resetEncoders();
		ahrs.gyroReset();
		
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
		
		enablePID();
		setStartPID();
	}

	protected void execute() {
		
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		
	}

	protected void interrupted() {
		
	}

}
