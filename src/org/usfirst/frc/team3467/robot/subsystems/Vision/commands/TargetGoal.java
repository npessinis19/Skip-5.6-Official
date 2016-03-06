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
	/*	x = grip.getCenterX();
		w = grip.getWidth();
		
		grip.isOnTarget(grip.calcDistnace(w), grip.calcAngle(x));
		*/
	}

	protected void execute() {
		x = grip.getCenterX();
		w = grip.getWidth();
		
		grip.isOnTarget(grip.calcDistnace(w), grip.calcAngle(x));
		
		SmartDashboard.putNumber("Vision: CenterX", x);
		SmartDashboard.putNumber("Vision: Width", w);
		SmartDashboard.putNumber("Vision: Distnace", grip.calcDistnace(w));
		SmartDashboard.putNumber("Vision: Angle", grip.calcAngle(x));
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
