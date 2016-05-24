package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.Robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionProfilingOld extends CommandBase {

	private MP_CANTalons leftmp_drive, rightmp_drive;
	private BuildTrajectory trajectory;
	
	private boolean m_reset;
	private double m_angle;
	private final double TOLERANCE = 1;
	
	/**
	 * Creates and runs a motion profile based on on input values.
	 * @param xnet Total distance in encoder ticks
	 * @param accel Acceleration in ticks per millisecond squared
	 * @param decel Deceleration in ticks per millisecond squared
	 * @param cruise Cruise velocity in ticks per millisecond
	 * @param step Time duration between each profile points millisecond
	 * @param reset Reset encoders before moving?
	 */
	public DriveMotionProfilingOld(int xnet, double accel, double decel, double cruise, int step, boolean reset) {
		requires(driveBase);
		
		this.m_reset = reset;
		
		setTimeout(10);
		
		trajectory = new BuildTrajectory(xnet, accel, decel, cruise, step);
		buildControllers();
	}
	
	/**
	 * Creates and runs a motion profile for turning to an angle based on input values
	 * 
	 * @param angle Angle of rotation (positive angle means a clockwise rotation)
	 * @param accel Acceleration in encoder ticks per millisecond squared
	 * @param decel Deceleration in ticks per millisecond squared
	 * @param cruise Cruise velocity in ticks per millisecond
	 * @param reset Reset encoders?
	 */
	public DriveMotionProfilingOld(double angle, double accel, double decel, double cruise, boolean reset) {
		requires(driveBase);
		
		this.m_angle = angle;
		this.m_reset = reset;
		
		setTimeout(10);
		
		trajectory = new BuildTrajectory((int) ((angle - 1) * 14), accel, decel, cruise, 10);
		buildControllers();
	}
	
	public void buildControllers() {
		leftmp_drive = new MP_CANTalons("Left Drive", driveBase.getLeftTalon(), false);
		rightmp_drive = new MP_CANTalons("Right Drive", driveBase.getRightTalon(), false);
	}
	
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			leftmp_drive.processMotionProfileBuffer();
			rightmp_drive.processMotionProfileBuffer();
		}
	}
	
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	
	public void startMP() {
		resetMP();
		
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
	
		leftmp_drive.changeMotionControlFramePeriod(20);
		rightmp_drive.changeMotionControlFramePeriod(20);

		notifier.startPeriodic(0.005);
		
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		leftmp_drive.upDateMotionProfileStatus();
		rightmp_drive.upDateMotionProfileStatus();
		
		SmartDashboard.putNumber("Init Left Bottom Buffer", leftmp_drive.BottomBufferCount());
		SmartDashboard.putNumber("Init Right Bottom Buffer", rightmp_drive.BottomBufferCount());
		
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
	}
	
	public void resetMP() {
		notifier.stop();
		
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		leftmp_drive.disableMotionProfiling();
		rightmp_drive.disableMotionProfiling();
	}
	
	public void publishValues() {
		leftmp_drive.upDateMotionProfileStatus();
		rightmp_drive.upDateMotionProfileStatus();
		
		SmartDashboard.putNumber("Left Top Buffer",leftmp_drive.topBuffercount());
		SmartDashboard.putNumber("Right Top Buffer", rightmp_drive.topBuffercount());
		
		SmartDashboard.putNumber("Left Bottom Buffer", leftmp_drive.BottomBufferCount());
		SmartDashboard.putNumber("Right Bottom Buffer", rightmp_drive.BottomBufferCount());
		
		SmartDashboard.putBoolean("Left Is Underrun", leftmp_drive.isUnderrun());
		SmartDashboard.putBoolean("Right Is Underrun", rightmp_drive.isUnderrun());
		
		SmartDashboard.putBoolean("Left Has Underrun", leftmp_drive.hasUnderrun());
		SmartDashboard.putBoolean("Right Has Underrun", rightmp_drive.hasUnderrun());
	
		SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
	}
	
	public void TalonOutput() {
		if (m_angle < 0) {
			driveBase.getLeftTalon().reverseOutput(false);
			driveBase.getRightTalon().reverseOutput(false);
		}
		else if (m_angle > 0){
			driveBase.getLeftTalon().reverseOutput(true);
			driveBase.getRightTalon().reverseOutput(true);
		}
		else {
			driveBase.getLeftTalon().reverseOutput(true);
			driveBase.getRightTalon().reverseOutput(false);
		}
	}
	
	
	protected void initialize() {
		if (m_reset) {
			driveBase.resetEncoders();
			ahrs.gyroReset();
		}
		
		TalonOutput();
		
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		driveBase.setTalonBrakes(false);
		driveBase.setSlaveMode(true);
		
		startMP();
	}
	
	protected void execute() {
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
		
		driveBase.reportEncoders();
		ahrs.reportGyroValues();
		publishValues();
	}
	
	protected boolean isFinished() {
		double error = Math.abs(m_angle - ahrs.getGyroAngle());
		boolean complete = (leftmp_drive.isComplete() || rightmp_drive.isComplete());
		
		return ( error <= TOLERANCE|| complete || isTimedOut());
	}

	protected void end() {
		driveBase.initDrive();
		resetMP();
	}

	protected void interrupted() {
	}
}

