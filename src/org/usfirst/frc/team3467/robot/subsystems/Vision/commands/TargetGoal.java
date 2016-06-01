package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	private static final boolean debugging = false;
	
	public TargetGoal() {
		requires(camera);
		System.out.println("Targeting Goal");
	}
	
	protected void initialize() {	
	}

	protected void execute() {
		camera.convert();
		camera.publishValues();
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
