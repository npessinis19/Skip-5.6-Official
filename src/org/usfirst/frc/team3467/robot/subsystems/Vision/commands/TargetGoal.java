package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	double x;
	double w;
	double angle;
	
	public TargetGoal() {
		setTimeout(1);
	}
	
	protected void initialize() {
		grip.createImage();
	}

	protected void execute() {
		x = grip.getCenterX();
		w = grip.getWidth();
		
		grip.isOnTarget();
	}

	protected boolean isFinished() {
		return isTimedOut();
	}

	protected void end() {
	}

	@Override
	protected void interrupted() {
		end();
	}

}
