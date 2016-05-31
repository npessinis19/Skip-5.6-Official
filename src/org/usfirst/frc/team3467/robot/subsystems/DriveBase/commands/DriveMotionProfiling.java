package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionProfiling extends CommandBase {

	private BuildTrajectory trajectory;
	
	private static boolean debugging = true;
	private static double TOLERANCE = 0.5;
	private double m_angle = 0.0;
	private boolean m_reset;
	
	
	/** Use this command to generate and drive a motion profile for both the left, and right CANTalons
	 * 
	 * @param xnet Distance in encoder ticks
	 * @param accel Acceleration in ticks per millisecond squared
	 * @param Deceleration in ticks per millisecond squared
	 * @param Cruise Velocity in ticks per millisecond
	 * @param Period between each profile point in milliseconds
	 * @param Reset Encoders before using profile?
	 */
	public DriveMotionProfiling(int xnet, double accel, double decel, double cruise, double step, boolean reset) {
		requires(driveBase);
		
		m_reset = reset;
		this.setInterruptible(true);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
		
		trajectory = new BuildTrajectory(xnet, accel, decel, cruise, step);
	}
 
	/**
	 * Create and drive a motion profile based on an input angle
	 *  
	 * @param Angle of rotation in degrees
	 * @param accel Acceleration in encoder ticks per millisecond squared
	 * @param decel Deceleration in ticks per millisecond squared
	 * @param Cruise velocity in ticks per millisecond
	 * @param Reset Encoders before using profile?
	 */
	public DriveMotionProfiling(double angle, double accel, double decel, double cruise, boolean reset) {
		requires(driveBase);
		
		m_angle = angle;
		m_reset = reset;
		this.setInterruptible(false);
		
		//setTimeout(10);
		
		SmartDashboard.putString("TestProfiling Mode", "angle");
		
		trajectory = new BuildTrajectory((int) (angle - 1) * 14, accel, decel, cruise, 10);
	}

	
	//Separate Thread that Processes points to Bottom Buffer
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			driveBase.processMotionProfileBuffer();
		}
	}
	
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	
	protected void initialize() {
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		if(m_reset) driveBase.resetEncoders();
		
		driveBase.getLeftTalon().reverseOutput(true);
		driveBase.getRightTalon().reverseOutput(true);
		
		ahrs.gyroReset();
		
		driveBase.startMP(trajectory);
		
		notifier.startPeriodic(0.001);
	}

	protected void execute() {
		ahrs.reportGyroValues();
		driveBase.reportEncoders();
		
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		
		//Sloppily turn on Motion Profile Executer
		driveBase.enableMP();
		
		driveBase.updateMotionProfileStatus();
		
		driveBase.publishValues();
		
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
		
		notifier.stop();
		
		driveBase.initDrive();
		
		System.out.println("Drive Motion Profiling Finished");
	}

	protected void interrupted() {
		end();
	}
}
