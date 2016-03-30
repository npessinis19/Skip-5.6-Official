package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	private PIDController vision_PID;
	private PIDF_CANTalon leftpidf_drive;
	private PIDF_CANTalon rightpidf_drive;
	private int count;
	
	private static final double P_P = 1.0;
	private static final double P_I = 0.0;
	private static final double P_D = 0.0;
	private static final double TOLERANCE = 1.0;

	private static final double P = 1.0;
	private static final double I = 0.0;
	private static final double D = 0.0;
	private static final double tolerance = 1.0;
	double position;
	
	boolean onTarget;
	
	public TargetGoal() {
	//	Build_Controller();
		setTimeout(4);
	}
	
	public void Build_Controller() {
		vision_PID = new PIDController(P, I, D, 
				new PIDSource() {
					PIDSourceType vision_SourceType = PIDSourceType.kDisplacement;
				
				public double pidGet() {
					return ahrs.getGyroAngle();
				}
				
				public void setPIDSourceType(PIDSourceType VisionTargetingPID) {
					vision_SourceType = VisionTargetingPID;
				}
				
				public PIDSourceType getPIDSourceType(){
					return vision_SourceType;
				}
				
		},
		new PIDOutput() {
				public void pidWrite(double output) {
					position = output * 13.75;
					
					leftpidf_drive.setSetpoint(position / 2);
					rightpidf_drive.setSetpoint(-position / 2);
				}});
		vision_PID.setAbsoluteTolerance(tolerance);
		
		leftpidf_drive = new PIDF_CANTalon("Left Drive", driveBase.getLeftTalon(), TOLERANCE, false, false);
		leftpidf_drive.setPID(P_P, P_I, P_D);
		
		rightpidf_drive = new PIDF_CANTalon("Right Drive",  driveBase.getRightTalon(), TOLERANCE, false, false );
		rightpidf_drive.setPID(P_P, P_I, P_D);
	}
	
	public void Set_Angle() {
		vision_PID.setSetpoint(grip.getChangeinAngle());
	}
	
	public void Start_PID() {
		vision_PID.enable();
		
		leftpidf_drive.enable();
		rightpidf_drive.enable();
	}
	
	public void reset_PID() {
		vision_PID.reset();
		
		leftpidf_drive.reset();
		rightpidf_drive.reset();
	}
	
	public void End_PID() {
		vision_PID.disable();
		
		leftpidf_drive.disable();
		rightpidf_drive.disable();
	}
	
	protected void initialize() {
		count = 0;
		
		ahrs.gyroReset();
		driveBase.resetEncoders();
		
		//driveBase.setSlaveMode(false);
	//	driveBase.setTalonBrakes(true);
		
		//reset_PID();
		//Start_PID();
	}

	protected void execute() {
		
		while ((!grip.isOnTarget() && count <= 10) || (!grip.isGoodImage() && count <= 5)) {
			grip.createImage();
			count = count + 1;
		}
		grip.calculateTargetData();
		grip.printData();
		
		//Set_Angle();
		
	SmartDashboard.putNumber("Vision: Change in Angle", grip.getChangeinAngle());
	SmartDashboard.putNumber("Vision: Change in Distance", grip.getChangeinDistance());
	}

	protected boolean isFinished() {
		//return isTimedOut() || grip.isOnTarget();
		return isTimedOut();
	}

	protected void end() {
		//End_PID();
		//reset_PID();
	}

	protected void interrupted() {
		end();
	}

}
