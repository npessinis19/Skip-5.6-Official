package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.TalonBuffer;
import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionProfiling extends CommandBase {

	private BuildTrajectory trajectory;
	private MP_CANTalons leftmp_drive, rightmp_drive;
	private Notifier notifier;
	
	public DriveMotionProfiling(int xnet, double accel, double decel, double cruise, double step) {
		requires(driveBase);
		buildManagers(xnet, step, step, step, xnet);
		
		this.setInterruptible(false);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
	}
 
	public DriveMotionProfiling(double angle, double accel, double decel, double cruise) {
		requires(driveBase);
		buildManagers((int) angle * 25, accel, decel, cruise, 10);
		
		this.setInterruptible(false);
		
		setTimeout(20);
		
		SmartDashboard.putString("TestProfiling Mode", "angle");
	}
	
	//Builds all controllers and managing objects
	public void buildManagers(int xnet, double accel, double decel, double cruise, int step) {
		leftmp_drive = new MP_CANTalons("Left Drive", driveBase.getLeftTalon(), false);
		rightmp_drive = new MP_CANTalons("Right Drive", driveBase.getRightTalon(), false);
		
		trajectory = new BuildTrajectory(xnet, accel, decel, cruise, step);
		notifier = new Notifier( new TalonBuffer(leftmp_drive, leftmp_drive));
	}

	
	public void startMP() {
		SmartDashboard.putString("TestProfiling Message", "startMP Called");
		
		resetMP();
		
		System.out.println("Starting Motion Profiling");
		
		leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount());
		rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount());
		
		/* Test Profiling
		 * leftmp_drive.testProfile();
		 * rightmp_drive.testProfile();
		 */
		
		leftmp_drive.changeMotionControlFramePeriod(20);
		rightmp_drive.changeMotionControlFramePeriod(2);
		
		notifier.startPeriodic(.005);
				
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		leftmp_drive.upDateMotionProfileStatus();
		
		SmartDashboard.putNumber("Init Botttom Buffer", leftmp_drive.ButtomBufferCount());

		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
	}
	
	public void resetMP() { 
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		leftmp_drive.disableMotionProfiling();
		rightmp_drive.disableMotionProfiling();
		
		System.out.println("MP Reset");
	}

	
	protected void initialize() {
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		driveBase.resetEncoders();
		ahrs.gyroReset();
		
		startMP();
	}

	protected void execute() {
		ahrs.reportGyroValues();
		driveBase.reportEncoders();
		
		driveBase.setControlMode(TalonControlMode.MotionProfile);
		leftmp_drive.upDateMotionProfileStatus();
		leftmp_drive.testExecuteOutput();
		
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
		
	//leftmp_drive.stallOnLastPoint();
	//	rightmp_drive.stallOnLastPoint();
		
		SmartDashboard.putNumber("Top Buffer", leftmp_drive.TopbufferCount());
		SmartDashboard.putNumber("Buttom Buffer", leftmp_drive.ButtomBufferCount());
		SmartDashboard.putBoolean("Is Underrun", leftmp_drive.getStatus().isUnderrun);
		SmartDashboard.putBoolean("Has Underrun", leftmp_drive.hasUnderrun());
		SmartDashboard.putBoolean("Is buffer full", leftmp_drive.isTopBufferFull());
		SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
		System.out.println("Active Point " + leftmp_drive.getActivePoint().position + " Time " + leftmp_drive.getActivePoint().timeDurMs);
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		resetMP();
	}

	protected void interrupted() {
		end();
	}
}
