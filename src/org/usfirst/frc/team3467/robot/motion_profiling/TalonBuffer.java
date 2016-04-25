package org.usfirst.frc.team3467.robot.motion_profiling;

import java.lang.Runnable;

import org.usfirst.frc.team3467.robot.RobotMap;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.DriveBase;
import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Notifier;

public class TalonBuffer implements java.lang.Runnable {
	
	private static MP_CANTalons left_drive, right_drive;
	
	public TalonBuffer(MP_CANTalons leftController, MP_CANTalons rightController) {
		left_drive = leftController;
		right_drive = rightController;
	}
	
	public void run(){
		left_drive.processMotionProfileBuffer();
		right_drive.processMotionProfileBuffer();
	}
}

