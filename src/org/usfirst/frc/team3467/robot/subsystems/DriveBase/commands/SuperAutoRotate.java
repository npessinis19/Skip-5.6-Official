
package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SuperAutoRotate extends CommandBase {

	private PIDController	GyroPID;;
	private PIDF_CANTalon  leftpidf_drive;
	private PIDF_CANTalon  rightpidf_drive;
	
	//PID Constant for Position
	private static double Posit_P = 4.0;
	private static double Posit_I = 0.0;
	private static double Posit_D = 0.0;
	private static double Posit_TOLERANCE = 5.0;
	double  position = 0.0;
	
	//PID Constants for Angle
	private static double Gyro_P = 1.0;
	private static double Gyro_I = 0.001;
	private static double Gyro_D = 0.0;
	private static double TOLERANCE = 0.05;
	double angle;
	int mode = 0;
	
	public SuperAutoRotate(double angle, int mode) {
		requires(driveBase);
		buildControllers();
		setTimeout(20);
		
		this.angle = angle;
		this.mode = mode;
	}
	
	public SuperAutoRotate(double angle, int mode, double p, double i, double d) {
		requires(driveBase);
		this.angle = angle;
		this.mode = mode;
		
		Gyro_P = p;
		Gyro_I = i;
		Gyro_D = d;
		
		buildControllers();
	}

	public SuperAutoRotate(double angle, int mode, double Gp, double Gi, double Gd,
													double Pp, double Pi, double Pd) {
		requires(driveBase);
		
		this.angle = angle;
		this.mode = mode;
		
		Gyro_P = Gp;
		Gyro_I = Gi;
		Gyro_D = Gd;
		
		Posit_P = Pp;
		Posit_I = Pi;
		Posit_D = Pd;
	}
	
	public void buildControllers() {
		
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
					position = angle * 14.75; //Encoder ticks required to rotate 1 degree
				
					switch(mode) {
						case 1:
								leftpidf_drive.setSetpoint(position);
							break;
						case 2:
								rightpidf_drive.setSetpoint(position);
							break;
						default:
								//Note, right encoder is backwards
								leftpidf_drive.setSetpoint(-position/2);
								rightpidf_drive.setSetpoint(-position/2);
							break;
					}
				}});
		
		GyroPID.setAbsoluteTolerance(TOLERANCE);
		
		leftpidf_drive = new PIDF_CANTalon("Left Position", driveBase.getLeftTalon(), Posit_TOLERANCE, false, false);
		leftpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
	
		rightpidf_drive = new PIDF_CANTalon("Right Position", driveBase.getRightTalon(), Posit_TOLERANCE, false, false);
		rightpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
	}
	
	public void startPID() {
		GyroPID.setSetpoint(angle);
		GyroPID.enable();
		
		leftpidf_drive.enable();
		rightpidf_drive.enable();
	}
	
	public void stopPID() {
		GyroPID.disable();
		leftpidf_drive.disable();
		rightpidf_drive.disable();
		
		GyroPID.reset();
		rightpidf_drive.reset();
		leftpidf_drive.reset();
	}
	
	protected void initialize() {
		ahrs.gyroReset();
		driveBase.resetEncoders();
		driveBase.setSlaveMode(false);
		driveBase.setControlMode(TalonControlMode.Position);
		startPID();
	}

	protected void execute() {
		driveBase.reportEncoders();
		SmartDashboard.putNumber("Gyro Error", GyroPID.getError());
		SmartDashboard.putNumber("Position (Rotate)", position);
	}

	protected boolean isFinished() {
		return GyroPID.onTarget() || isTimedOut();
	}

	protected void end() {
		stopPID();
		driveBase.initDrive();
	}

	protected void interrupted() {
		end();
	}

}
