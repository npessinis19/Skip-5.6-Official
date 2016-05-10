package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	private static final boolean debugging = false;
	
	public TargetGoal() {
		System.out.println("Targeting Goal");
		setTimeout(0.5);
	}
	
	protected void initialize() {	
		grip.createImage();
		grip.calculateTargetData();
	}

	protected void execute() {
		grip.printData();
		
		SmartDashboard.putNumber("Vision: Change in Angle", grip.getChangeinAngle());
		SmartDashboard.putNumber("Vision: Change in Distance", grip.getChangeinDistance());
		SmartDashboard.putBoolean("Vision: Image on Target", grip.isOnTarget());
	}

	protected boolean isFinished() {
		return isTimedOut();
	}

	protected void end() {
	}

	protected void interrupted() {
		end();
	}
}
