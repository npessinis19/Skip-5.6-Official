package org.usfirst.frc.team3467.robot.subsystems.Vision;

import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.TargetGoal;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PixyCam extends Subsystem {

	private double angle = 0.0;
	private double changeinAngle = 0.0;
	private static double targetAngle = 0.0;
	
	private static AnalogInput Pixy;
	
	Preferences m_prefs = Preferences.getInstance();
	
	public PixyCam(){
		Pixy = new AnalogInput(7);
		targetAngle = m_prefs.getDouble("Ideal Angle", 0);
	}
	
	public static int read(){
		return Pixy.getValue();
	}
	
	public AnalogInput getPixy(){
		return Pixy;
	}
	
	public void publishValues() {
		SmartDashboard.putNumber("Pixy Angle", changeinAngle); 
		SmartDashboard.putNumber("Pixy Raw", read());
	}
	
	public double convert(){
		angle = (80 / (2653.0 - 70)) * (read() - 70.0) - 37.2;
		changeinAngle = angle;
		return changeinAngle;
	}
	
	public void setTargetAngle() {
		targetAngle = angle = (80 / (2653.0 - 70.0)) * (read() - 70.0) - 35.0;
		m_prefs.putDouble("Ideal Angle", targetAngle);
	}

	@Override
	protected void initDefaultCommand() {
		this.setDefaultCommand(new TargetGoal());
	}
	
}