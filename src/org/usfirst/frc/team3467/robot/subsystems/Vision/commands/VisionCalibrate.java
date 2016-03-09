package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionCalibrate extends CommandBase {

	public VisionCalibrate() {
		setTimeout(0.5);
	}
	
	protected void initialize() {
		grip.createImage();
		grip.setTarget_angle(grip.getAngle_theta());
		grip.setTarget_distnce(grip.getDistance_delta());
	}
	
	protected void execute() {
		grip.isOnTarget();
		SmartDashboard.putNumber("Target Distance", grip.changeinDistance);
		SmartDashboard.putNumber("Target Angle", grip.changeinAngle);
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
