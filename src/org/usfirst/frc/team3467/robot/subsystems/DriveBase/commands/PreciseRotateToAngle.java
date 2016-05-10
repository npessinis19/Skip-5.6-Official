package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PreciseRotateToAngle extends CommandBase {

	double PreciseInput;
	
	public PreciseRotateToAngle() {
		requires(driveBase);
	}
	
	protected void initialize() {
		driveBase.setControlMode(TalonControlMode.Voltage);
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
	}

	protected void execute() {
//		PreciseInput = oi.getPrimeTwist()/5;
		PreciseInput = oi.getPrimeTwist() * 6;
		
		driveBase.getLeftTalon().set(-PreciseInput);
		driveBase.getRightTalon().set(-PreciseInput);
		
		//driveBase.driveTank(-PreciseInput, PreciseInput, false);
		SmartDashboard.putNumber("Gyro Angle", ahrs.getGyroAngle());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		driveBase.driveArcade(0, 0, false);
		driveBase.initDrive();
	}

	protected void interrupted() {
		end();
	}
}
