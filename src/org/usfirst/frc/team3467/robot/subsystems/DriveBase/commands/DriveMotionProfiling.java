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
		buildControllers();
		
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
		buildControllers();
		
		m_angle = angle;
		m_reset = reset;
		this.setInterruptible(false);
		
		setTimeout(10);
		
		SmartDashboard.putString("TestProfiling Mode", "angle");
		
		trajectory = new BuildTrajectory((int) (angle - 1) * 14, accel, decel, cruise, 10);
	}
	
	
	public void buildControllers() {
		leftmp_drive = new MP_CANTalons("Left Drive", driveBase.getLeftTalon(), false);
		rightmp_drive = new MP_CANTalons("Right Drive", driveBase.getRightTalon(), false);
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
	
	public void startMP() {
		SmartDashboard.putString("TestProfiling Message", "startMP Called");
		
		resetMP();
		
		System.out.println("Starting Motion Profiling");
		
		leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		
		//leftmp_drive.testProfile();
		//rightmp_drive.testProfile();
		
		leftmp_drive.changeMotionControlFramePeriod(20);
		rightmp_drive.changeMotionControlFramePeriod(20);
		
		notifier.startPeriodic(.005);
				
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		leftmp_drive.upDateMotionProfileStatus();
		rightmp_drive.upDateMotionProfileStatus();
		
		SmartDashboard.putNumber("Init Left Botttom Buffer", leftmp_drive.BottomBufferCount());
		SmartDashboard.putNumber("Init Right Bottom Buffer", rightmp_drive.BottomBufferCount());
		
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
	}
	
	public void resetMP() { 
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		leftmp_drive.disableMotionProfiling();
		rightmp_drive.disableMotionProfiling();
		
		notifier.stop();
		
		System.out.println("MP Reset");
	}

	public void publishValues() {
		if (debugging) {
			leftmp_drive.upDateMotionProfileStatus();
			rightmp_drive.upDateMotionProfileStatus();
			
			SmartDashboard.putNumber("Left Top Buffer", leftmp_drive.TopbufferCount());
			SmartDashboard.putNumber("Right Top Buffer", rightmp_drive.TopbufferCount());
			
			SmartDashboard.putNumber("Left Bottom Buffer", leftmp_drive.BottomBufferCount());
			SmartDashboard.putNumber("Right Bottom Buffer", rightmp_drive.BottomBufferCount());
			
			SmartDashboard.putBoolean("Left Is Underrun", leftmp_drive.isUnderrun());
			SmartDashboard.putBoolean("Right Is Underrun", rightmp_drive.isUnderrun());
			
			SmartDashboard.putBoolean("Left Has Underrun", leftmp_drive.hasUnderrun());
			SmartDashboard.putBoolean("Right Has Underrrun", rightmp_drive.hasUnderrun());
			
			System.out.println("Active Point Left " + leftmp_drive.getActivePoint().position + " Time " + leftmp_drive.getActivePoint().timeDurMs);
			System.out.println("Active Point Right " + rightmp_drive.getActivePoint().position + " Time " + rightmp_drive.getActivePoint().timeDurMs);
		}
	}
	
	
	protected void initialize() {
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		if(m_reset) driveBase.resetEncoders();
		
		driveBase.getLeftTalon().reverseOutput(true);
		driveBase.getRightTalon().reverseOutput(true);
		
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		ahrs.gyroReset();
		
		startMP();
	}

	protected void execute() {
		ahrs.reportGyroValues();
		driveBase.reportEncoders();
		
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		leftmp_drive.upDateMotionProfileStatus();
		
		publishValues();
		
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
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		resetMP();
		
		driveBase.initDrive();
		
		System.out.println("Drive Motion Profiling Finished");
	}

	protected void interrupted() {
		end();
	}
}
