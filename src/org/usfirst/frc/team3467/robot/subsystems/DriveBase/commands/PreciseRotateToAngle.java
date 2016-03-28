package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PreciseRotateToAngle extends CommandBase {

	double PreciseInput;
	
	public PreciseRotateToAngle() {
		requires(driveBase);

	}
	
	protected void initialize() {
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
	}

	protected void execute() {
//		PreciseInput = oi.getPrimeTwist()/5;
		PreciseInput = oi.getPrimeTwist()/4.5;
		driveBase.driveTank(-PreciseInput, PreciseInput, false);
		SmartDashboard.putNumber("Gyro Angle", ahrs.getGyroAngle());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		driveBase.driveArcade(0, 0, false);
	}

	protected void interrupted() {
		end();
	}
}
