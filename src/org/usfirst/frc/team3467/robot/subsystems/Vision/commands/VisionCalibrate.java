package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionCalibrate extends CommandBase {

	public VisionCalibrate() {
		setTimeout(5);
		this.setInterruptible(true);
	}
	
	protected void initialize() {
		grip.createImage();
		grip.calculateTargetData();
		
		grip.setTarget_angle(grip.getAngle_theta());
		grip.setTarget_distnce(grip.getDistance_delta());
	}
	
	protected void execute() {
	//	if (!grip.createImage()) {
		//	return;
	//	};
		SmartDashboard.putNumber("Target Distance", grip.getChangeinDistance());
		SmartDashboard.putNumber("Target Angle", grip.getChangeinAngle());
	}

	protected boolean isFinished() {
		return isTimedOut();
	}

	protected void end() {
		light.lightOn();
	}

	protected void interrupted() {
		end();
	}

}
