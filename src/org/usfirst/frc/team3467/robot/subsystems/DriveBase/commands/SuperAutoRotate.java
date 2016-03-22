package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperAutoRotate extends CommandBase {

	//Test Comment
	
	private PIDController	GyroPID;;
	private PIDF_CANTalon  leftpidf_drive;
	private PIDF_CANTalon rightpidf_drive;
	
	//PID Variables for Position
	private static final double Posit_P = 0.0;
	private static final double Posit_I = 0.0;
	private static final double Posit_D = 0.0;
	private static final double Posit_TOLERANCE = 0.0;
	double Position = 0.0;
	
	//PID Constants for Angle
	private static final double Gyro_P = 0.0;
	private static final double Gyro_I = 0.0;
	private static final double Gyro_D = 0.0;
	private static final double Gyro_TOLERANCE = 0.0;
	double angle;
	int mode = 0;
	
	public SuperAutoRotate(double angle, int mode) {
		requires(driveBase);
		buildGyroController();
		this.angle = angle;
		this.mode = mode;
	}
	
	public void buildGyroController() {
		GyroPID = new PIDController(Gyro_P, Gyro_I, Gyro_D, 
				new PIDSource() {
					PIDSourceType m_gyroSourceType = PIDSourceType.kDisplacement;
			
				public double pidGet() {
					return ahrs.getGyroYaw();
				}
				
				public void setPIDSourceType(PIDSourceType pidSource) {
					m_gyroSourceType = pidSource;
				}
				
				public PIDSourceType getPIDSourceType() {
					return m_gyroSourceType;
				}
		}, 
		new PIDOutput() {
				
				public void pidWrite(double angle) {
					//Calculates the output of the motor needed to rotate a certain number of degrees
					Position = angle;
				}});	
		
		GyroPID.setAbsoluteTolerance(Gyro_TOLERANCE);
		GyroPID.setSetpoint(angle);
	}

	public void buildPositController() {
		leftpidf_drive = new PIDF_CANTalon("Left Position", driveBase.getLeftTalon(), Posit_TOLERANCE, false, false);
		leftpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
		
		rightpidf_drive = new PIDF_CANTalon("Right Position", driveBase.getRightTalon(), Posit_TOLERANCE, false, false);
		rightpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
		
		switch (mode) {
		case 1:
						leftpidf_drive.setSetpoint(Position);
			break;
		case 2:
						rightpidf_drive.setSetpoint(-Position);
			break;
		default:
						leftpidf_drive.setSetpoint(Position);
						rightpidf_drive.setSetpoint(-Position);
			break;
		}
	}
	
	public void startPID() {
		GyroPID.enable();
		leftpidf_drive.enable();
		rightpidf_drive.enable();
	}
	
	public void stopPID() {
		GyroPID.disable();
		GyroPID.reset();
		
		leftpidf_drive.disable();
		leftpidf_drive.reset();
		rightpidf_drive.disable();
		rightpidf_drive.reset();
	}
	
	public void resetPID() {
		GyroPID.reset();
		leftpidf_drive.reset();
		rightpidf_drive.reset();
	}
	
	protected void initialize() {
		driveBase.setSlaveMode(false);
		driveBase.setControlMode(TalonControlMode.Position);
		
		//Get everything in safe starting state
		resetPID();
		startPID();
	}

	protected void execute() {
		SmartDashboard.putNumber("Gyro Angle", ahrs.getGyroAngle());
		driveBase.reportEncoders();
	}

	protected boolean isFinished() {
		double error = GyroPID.getError();
		return (error >= 0 && error <= Gyro_TOLERANCE) || isTimedOut() || GyroPID.onTarget();
	}

	protected void end() {
		stopPID();
	}

	protected void interrupted() {
		end();
	}
	
	

}
