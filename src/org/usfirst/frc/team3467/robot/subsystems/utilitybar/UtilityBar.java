package org.usfirst.frc.team3467.robot.subsystems.utilitybar;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.usfirst.frc.team3467.robot.RobotMap;

public class UtilityBar extends Subsystem {
	
	//Constants used for OI
	public static final Value kOut = Value.kForward;
	public static final Value kIn = Value.kReverse;
	public static final Value kOff = Value.kOff;
	
	//Objects in UtilityBar
	public static DoubleSolenoid barSolenoid;
	public static DoubleSolenoid fingerSolenoid;
	private static UtilityBar instance; 
	
	//Gets instance of UtiltiyBar System
	public UtilityBar getInstance() {
		return instance;
	}
	
	//Constructor method for UtilityBar class
	public UtilityBar() {
		instance = this;
		
		barSolenoid = new DoubleSolenoid(RobotMap.utilitybar_solenoid_deploy,
											RobotMap.utilitybar_solenoid_retract);
		fingerSolenoid = new DoubleSolenoid(RobotMap.utilityfinger_solenoid_out, 
											RobotMap.utilityfinger_solenoid_in);
	}
	
	//Use Class Constants
	public void setBar(Value actuate) {
		barSolenoid.set(actuate);
	}
	
	public void setFinger(Value actuate) {
		fingerSolenoid.set(actuate);
	}
	
	public Value getBarState() {
		return barSolenoid.get();
	}
	
	public Value getFingerState() {
		return fingerSolenoid.get();
	}

	protected void initDefaultCommand() {
	}
}