package org.usfirst.frc.team3467.robot.subsystems.BallEject;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.usfirst.frc.team3467.robot.RobotMap;

public class BallEject extends Subsystem {
	
	public static final Value kOut = Value.kForward;
	public static final Value kIn = Value.kReverse;
	public static final Value kOff = Value.kOff;
	
	
	public static DoubleSolenoid Eject;
	
	private static BallEject instance;
	
	public BallEject getInstance() {
		return instance;
		
}
	public BallEject() {
		instance = this;
		
		 Eject = new DoubleSolenoid(RobotMap.BallEject_solenoid_deploy,
				 					RobotMap.BallEject_solenoid_retract);
}
	public static void setBar(Value actuate) {
		BallEject.setBar(actuate);

	}
	protected void initDefaultCommand() {
	}
}