
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
	private static final double Posit_TOLERANCE = 5.0;
	private double  position = 0.0;
	
	//PID Constants for Angle
	private static double Gyro_P = 1.0;
	private static double Gyro_I = 0.001;
	private static double Gyro_D = 0.0;
	private static final double Gyro_TOLERANCE = 0.5;
	
	//Management Variables
	private double angle; //Desired angle as SetPoint
	private int mode = 0; //Turning mode (0-both sides, 1-left side, 2-right side)
	private boolean stop = false; //Turn off command when in tolerance zone
	private int count = 0; //Counter starts when robot is in tolerance zone
	private static final int sufficientCount = 28; //Once count reaches 28, stop command
	private double oldGyro_P = Gyro_P; //Storing previous value of Gyro_P
	
	private static final boolean debugging = true;
	
	public SuperAutoRotate(double angle, int mode) {
		requires(driveBase);
		
		buildControllers();
		setTimeout(20);
		
		this.angle = angle;
		this.mode = mode;
		
		this.setInterruptible(true);
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
					double error = GyroPID.getError();
					
					if (Math.abs(error) >= 0 || Math.abs(error) <= Gyro_TOLERANCE) {
						if (Gyro_P > 0.1) {
							Gyro_P = oldGyro_P - 0.3 * (count + 1);
							if (Gyro_P < 0.01) Gyro_P = 0.01;
							
							GyroPID.setPID(Gyro_P, Gyro_I, Gyro_D);
							oldGyro_P = Gyro_P;
						}
						
						System.out.println("Gyro_P " + Gyro_P);
						System.out.println("count " + count);
						
						if (count++ < sufficientCount) {
							stop = true;
						}
					}
					else {
						count = 0;
							
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
							}
				}});
		
		GyroPID.setAbsoluteTolerance(Gyro_TOLERANCE);
		
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
	
	public void deBug() {
		if (debugging) {
			double G_p = SmartDashboard.getNumber("SuperRotate: P");
			double G_i = SmartDashboard.getNumber("SuperRotate: I");
			double G_d = SmartDashboard.getNumber("SuperRotate: D");
			
			//Manualy Control Gyro_P?
			//GyroPID.setPID(G_p, G_i, G_d);;
			GyroPID.setPID(Gyro_P, G_i, G_d);	
			
			SmartDashboard.putNumber("SuperRotate: Gyro Error", GyroPID.getError());
			SmartDashboard.putNumber("SuperRotate", GyroPID.getSetpoint());
			SmartDashboard.putNumber("SuperRotate: Position", position);
		}
	}
	
	
	protected void initialize() {
		ahrs.gyroReset();
		driveBase.resetEncoders();
		
		driveBase.setSlaveMode(false);
		driveBase.setControlMode(TalonControlMode.Position);
		
		if (debugging) {
			SmartDashboard.putNumber("SuperRotate: P", Gyro_P);
			SmartDashboard.putNumber("SuperRotate: I", Gyro_I);
			SmartDashboard.putNumber("SuperRotate: D", Gyro_D);
		}
		
		stopPID();
		startPID();
	}

	protected void execute() {
		driveBase.reportEncoders();
		ahrs.reportGyroValues();
		deBug();
	}

	protected boolean isFinished() {
		//return GyroPID.onTarget() || isTimedOut();
		return stop;
	}

	protected void end() {
		stopPID();
		driveBase.initDrive();
	}

	protected void interrupted() {
		end();
	}
}
