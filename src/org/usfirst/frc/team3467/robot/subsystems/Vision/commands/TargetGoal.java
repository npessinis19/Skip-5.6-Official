package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	private PIDController vision_PID;
	private PIDF_CANTalon leftpidf_drive;
	private PIDF_CANTalon rightpidf_drive;
	
	private static final double P_P = 0.0;
	private static final double P_I = 0.0;
	private static final double P_D = 0.0;
	private static final double TOLERANCE = 0.0;

	private static final double P = 0.0;
	private static final double I = 0.0;
	private static final double D = 0.0;
	private static final double tollerance = 0.0;
	double position;
	
	boolean onTarget;
	
	public TargetGoal() {
		setTimeout(4);
	}
	
	public void Build_Controller() {
		vision_PID = new PIDController(P, I, D, 
				new PIDSource() {
					PIDSourceType vision_SourceType = PIDSourceType.kDisplacement;
				
				public double PIDGet() {
					return ahrs.getGyroAngle();
				}
				
				public void SetSourceType(PIDSourceType VisionTargetingPID) {
					VisionSourceType = VisionTargetingPID;
				}
				
				public PIDSourceType GetSourceType(){
					return VisionTargetingPID;
				}
			},
		new PIDOutput() {
				
				public void pidWrite(double output){
					position = output * 13.75;
				}});
		
		leftpidf_drive = new PIDF_CANTalon("Left Drive", driveBase.getLeftTalon(), TOLERANCE, false, false);
		leftpidf_drive.setPID(P_P, P_I, P_D);
		
		rightpidf_drive = new PIDF_CANTalon("Right Drive",  driveBase.getRightTalon(), TOLERANCE, false, false );
		rightpidf_drive.setPID(P_P, P_I, P_D);
	}
	
	protected void initialize() {
	}

	protected void execute() {
		
		if (!grip.createImage()) {
			return;
		}
		
		onTarget = grip.isOnTarget();
		
	SmartDashboard.putNumber("Target Distance", grip.getChangeinAngle());
	SmartDashboard.putNumber("Target Angle", grip.getChangeinDistance());
	}

	protected boolean isFinished() {
		return isTimedOut() || onTarget;
	}

	protected void end() {
	}

	protected void interrupted() {
		end();
	}

}
