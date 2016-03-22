package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team3467.robot.OI;

public class PIDRate extends CommandBase {

	/*private PIDController AnglePIDController;
	//private static final double Angle_KP = 0.0;
	//private static final double Angle_KI = 0.0;
	//private static final double Angle_KD = 0.0;
	*/
	public PIDRate() {
		requires(driveBase);
	}
	/*
	public void BuildAnglePIDController() {
		AnglePIDController = new PIDController(Angle_KP, Angle_KI, Angle_KD);
		
		new PIDSource() {
            PIDSourceType m_sourceType = PIDSourceType.kDisplacement;
            
            public double pidGet() {
            	return ahrs.getGyroYaw();
            }
            
            public double pidSet() {
            	return ahrs.setGyroYaw();
            }
            
		}}
	*/
	protected void initialize() {
		
	}

	protected void execute() {
		
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		
	}

	protected void interrupted() {
		
	}

}
