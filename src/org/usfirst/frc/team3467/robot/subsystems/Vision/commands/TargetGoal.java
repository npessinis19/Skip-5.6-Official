package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	boolean onTarget;
	
	public TargetGoal() {
		setTimeout(1);
	}
	
	protected void initialize() {
	}

	protected void execute() {
		
		if (!grip.createImage()) {
			return;
		}
		
		onTarget = grip.isOnTarget();
		
		SmartDashboard.putNumber("Target Distance", grip.changeinDistance);
		SmartDashboard.putNumber("Target Angle", grip.changeinAngle);
	}

	protected boolean isFinished() {
		return isTimedOut() || onTarget;
	}

	protected void end() {
	}

	protected void interrupted() {
		end();
	}

}
