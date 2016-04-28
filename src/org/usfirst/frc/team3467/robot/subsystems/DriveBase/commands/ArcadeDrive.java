package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArcadeDrive extends CommandBase {
	
	public ArcadeDrive() {
		requires(driveBase);
		this.setInterruptible(true);
	}
	
	protected void initialize() {
		driveBase.initDrive();
	}

	protected void execute() {
		driveBase.driveArcade(oi.getPrimeY(), oi.getPrimeX(), true);
		
		// Alternate Method?
		//driveBase.driveArcade(oi.getPrimeY(), oi.getPrimeTwist(), true);
		
		ahrs.reportGyroValues();
		SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
	}

	protected void interrupted() {
		end();
	}

}
