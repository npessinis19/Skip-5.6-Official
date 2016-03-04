package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	double x;
	double y;
	double angle;
	double distance;
	
	public TargetGoal() {
		x = grip.getCenterX();
		y = grip.getCenterY();
	}
	
	protected void initialize() {
	}

	protected void execute() {
		SmartDashboard.putNumber("Vision: CenterX", x);
		SmartDashboard.putNumber("Vision: CenterY", y);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}

}
