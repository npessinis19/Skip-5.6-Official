package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;

import java.util.ArrayList;

public class DriveMotionProfiling extends CommandBase {

	private BuildTrajectory trajectory;
	private MP_CANTalons leftmp_drive, rightmp_drive;
	
	private static boolean debugging = true;
	private static double TOLERANCE = 0.5;
	private double m_angle = 0.0;
	private boolean m_reset;
	
	
	/**
	 * @param Distance
	 * @param Acceleration
	 * @param Deceleration
	 * @param Cruise Velocity
	 * @param Period
	 * @param Reset Encoders
	 */
	public DriveMotionProfiling(int xnet, double accel, double decel, double cruise, double step, boolean reset) {
		requires(driveBase);
		
		m_reset = reset;
		this.setInterruptible(true);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
		
		trajectory = new BuildTrajectory(xnet, accel, decel, cruise, step);
	}
 
	/**
	 * Drive a created motion profile based on input data
	 * 
	 * @param Angle of rotation
	 * @param Acceleration
	 * @param Deceleration
	 * @param Cruise velocity
	 * @param Reset Encoders 
	 */
	public DriveMotionProfiling(double angle, double accel, double decel, double cruise, boolean reset) {
		requires(driveBase);
		
		m_angle = angle;
		m_reset = reset;
		this.setInterruptible(false);
		
		setTimeout(10);
		
		SmartDashboard.putString("TestProfiling Mode", "angle");
		
		trajectory = new BuildTrajectory((int) (angle - 1) * 14, accel, decel, cruise, 10);
	}

	
	//Separate Thread that Processes points to Bottom Buffer
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			leftmp_drive.processMotionProfileBuffer();
			rightmp_drive.processMotionProfileBuffer();
		}
	}
	
	/*
	public DriveMotionProfiling(){
		leftmp_drive.changeMotionControlFramePeriod(1);
		//rightmp_drive.changeMotionControlFramePeriod(1);
		notifier.startPeriodic(.001);
	}
	*/
	
	Notifier notifier = new Notifier(new PeriodicRunnable());

	protected void initialize() {
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		if(m_reset) driveBase.resetEncoders();
		
		driveBase.getLeftTalon().reverseOutput(true);
		driveBase.getRightTalon().reverseOutput(true);
		
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		ahrs.gyroReset();
		notifier.startPeriodic(0.005);
		driveBase.startMP(trajectory);
	}

	protected void execute() {
		ahrs.reportGyroValues();
		driveBase.reportEncoders();
		
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		leftmp_drive.upDateMotionProfileStatus();
		
		driveBase.publishValues();
		
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
		
		//leftmp_drive.stallOnLastPoint();
		//rightmp_drive.stallOnLastPoint();
		
		SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
	}

	protected boolean isFinished() {
		double error = Math.abs(m_angle - ahrs.getGyroYaw());
		return isTimedOut() || error <= TOLERANCE;
	}

	protected void end() {
		driveBase.resetMP();
		
		driveBase.initDrive();
		
		System.out.println("Drive Motion Profiling Finished");
	}

	protected void interrupted() {
		end();
	}
}
