package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SuperAutoRotate extends CommandBase {

	//Test Comment
	
	private PIDController	GyroPID;;
	private PIDF_CANTalon  leftpidf_drive;
	
	//PID Constant for Position
	private static final double Posit_P = 0.0;
	private static final double Posit_I = 0.0;
	private static final double Posit_D = 0.0;
	private static final double Posit_TOLERANCE = 0.0;
	
	//PID Constants for Angle
	private static final double Gyro_P = 0.0;
	private static final double Gyro_I = 0.0;
	private static final double Gyro_D = 0.0;
	
	public SuperAutoRotate() {
		requires(driveBase);
		buildGyroController();
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
					double position = angle;
				}});	
	}

	public void buildPositController() {
		leftpidf_drive = new PIDF_CANTalon("Left Position", driveBase.getLeftTalon(), Posit_TOLERANCE, false, false);
		leftpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
	}
	
	protected void initialize() {
		driveBase.setSlaveMode(false);
		driveBase.setControlMode(TalonControlMode.Position);
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
