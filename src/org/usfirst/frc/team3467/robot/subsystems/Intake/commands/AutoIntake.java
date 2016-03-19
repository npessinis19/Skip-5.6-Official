package org.usfirst.frc.team3467.robot.subsystems.Intake.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.Intake.Intake;

public class AutoIntake extends CommandBase {

	boolean autoIntakeIn;
	
	public AutoIntake(boolean driveOut) {
		requires(intake);
		autoIntakeIn = driveOut;
		setTimeout(1);
	}
	
	protected void initialize() {
	}
	
	protected void execute() {
		if (autoIntakeIn) {
			intake.driveManual(Intake.kIntakeFast);
		}
		else {
			intake.driveManual((Intake.kEjectFast));
		}
	}

	protected boolean isFinished() {
		return isTimedOut();
	}

	protected void end() {
		intake.driveManual(0.0);
	}

	protected void interrupted() {
	}
}
