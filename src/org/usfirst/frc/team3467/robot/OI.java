package org.usfirst.frc.team3467.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.AimBot;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.LightSwitch;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.TargetGoal;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.VisionCalibrate;
import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.commands.autonomous.AutoTarget;
import org.usfirst.frc.team3467.robot.commands.autonomous.LowBarAndShoot;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.ArcadeDrive;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveMotionProfiling;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.PreciseRotateToAngle;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.ResetDriveEncoders;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.SetBrakeMode;
import org.usfirst.frc.team3467.robot.subsystems.Intake.Intake;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.IntakeDrive;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.NavX_MXP.command.ResetGyro;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.*;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.*;
import org.usfirst.frc.team3467.robot.control.Gamepad;
import org.usfirst.frc.team3467.robot.triggers.DPadDown;
import org.usfirst.frc.team3467.robot.triggers.DPadLeft;
import org.usfirst.frc.team3467.robot.triggers.DPadRight;
import org.usfirst.frc.team3467.robot.triggers.DPadUp;
import org.usfirst.frc.team3467.robot.triggers.DoubleButton;
import org.usfirst.frc.team3467.robot.triggers.GamepadLeftTrigger;
import org.usfirst.frc.team3467.robot.triggers.GamepadRightTrigger;

public class OI {
	
	public static Joystick PrimaryStick;
	public static Joystick SecondaryStick;
	public static Gamepad operator;
	
	//User numbers for different button layouts
	public static final int Tank = 1;
	public static final int Arcade = 2;
	public int userlogin = 2;
	
	
/*
 * Joystick Mappings (done elsewhere in code)
 * 
 * Joystick PrimaryStick - used for Tank or Arcade Drive
 * 
 * Gamepad getRightStickX() - used for manual drive of Intake rollers
 * Gamepad getLeftStickY() - used for manual drive of Catapult reset bar
 * 
 */
	
	
	public OI(){
		PrimaryStick = new Joystick(0);
		//SecondaryStick = new Joystick(1);
		operator = new Gamepad(2);
	}
	
	
	public Gamepad getGamepad() {
		return operator;
	}
	
	//Joystick Methods that return values for left and right joysticks
	public double getPrimeY(){
		return PrimaryStick.getY();
	}
	
	public double getSecondaryY(){
		return SecondaryStick.getY();
	}
	
	public double getPrimeX(){
		return PrimaryStick.getX();
	}
	
	public double getPrimeTwist() {
		return PrimaryStick.getTwist();
	}
	
	public int getUserlogin() {
		userlogin = (int) SmartDashboard.getNumber("Enter 1 for Tank, 2 for Arcade");
		return userlogin;
	}
	
	
	//Method that binds certain commands to certain buttons
	public void BindCommands() {

	// Set the drive mode	
		switch (userlogin) {
		case Tank: 
		default: 
				CommandBase.driveBase.setDriveMode(true);
				break; 
		case Arcade: 
				CommandBase.driveBase.setDriveMode(false);
				break;
		}

	//DriveBase
		//Toggle in and out of precision angle mode
		new JoystickButton(PrimaryStick, 3)
			.whenPressed(new PreciseRotateToAngle());
		
		new JoystickButton(PrimaryStick, 4)
			.whenPressed(new ArcadeDrive());
		
		//Toggle in and out of AimBot
		new JoystickButton(PrimaryStick, 1)
			.whenPressed(new AimBot());
		
		new JoystickButton(PrimaryStick, 2)
			.whenPressed(new ArcadeDrive());
		
	//Utility Bar
		//Utility bar up
		new GamepadLeftTrigger(operator)
		.whenActive(new Bar_actuate(UtilityBar.kOut));
		
		//Utility bar down
		new GamepadRightTrigger(operator)
			.whenActive(new Bar_actuate(UtilityBar.kIn));
	
		
	//Intake
		//Eject Fast
		new JoystickButton(operator, Gamepad.xButton)
			.whileHeld(new IntakeDrive(Intake.kEjectFast));
		
		//Intake Fast
		new JoystickButton(operator, Gamepad.bButton)
			.whileHeld(new IntakeDrive(Intake.kIntakeFast));
		
		//Intake up
		new JoystickButton(operator, Gamepad.aButton)
			.whenActive(new Roller_Actuate(true));
		
		//Intake down
		new JoystickButton(operator, Gamepad.yButton)
			.whenActive(new Roller_Actuate(false));
		
		/*
		//Intake Extend
		new JoystickButton(SecondaryStick, 1)
		.whenPressed(new Roller_Actuate(true));
		
		new JoystickButton(SecondaryStick, 2)
		.whenPressed(new Roller_Actuate(false));	
		*/
	
	//Catapult
		//Reload Catapult
		new JoystickButton(operator, Gamepad.leftBumper)
			.whenPressed(new ShooterSetup());
		
		//Fire Catapult
		new JoystickButton(operator, Gamepad.rightBumper)
			.whenPressed(new Shoot());
	
		
		// DPad Up
		new DPadUp(operator)
			.whenActive(new SetBrakeMode(false));

		// DPad Down
		new DPadDown(operator)
			.whenActive(new SetBrakeMode(true));
 		
		/*
		 DPad Right
		new DPadRight(operator)
			.whenActive(new ShooterLatch());

		DPad Left
		new DPadLeft(operator)
			.whenActive(new ShooterClear());
		 */
		
		// Quick latch reset (emergency use only)
		new JoystickButton(operator, Gamepad.leftStickPress)
		.whenActive(new ShooterRetryUnlatch());
		
		
	//Camera Commands
		new JoystickButton(operator, Gamepad.startButton)
		.whenPressed(new LightSwitch(true));
		
		new JoystickButton(operator, Gamepad.backButton)
		.whenActive(new LightSwitch(false));
		
	/*	
	//Utility Bar/Finger
			//Extend Using the Right Joystick Trigger
		new JoystickButton(SecondaryStick, 3)
			.whenPressed(new Bar_actuate(UtilityBar.kOut));
		
			//Retract Using the Right Shoulder Button
		new JoystickButton(SecondaryStick, 4)
			.whenPressed(new Bar_actuate(UtilityBar.kIn));
	*/
		
			//Extend Finger using Top left button
		/*
		 * new JoystickButton(PrimaryStick, 1)
			.whenPressed(new Finger_actuate(UtilityBar.kOut));
		
			//Retract Finger using Top Right Button
		new JoystickButton(PrimaryStick, 2)
			.whenActive(new Finger_actuate(UtilityBar.kIn));
		*/
		
		// SmartDashboard Buttons
		SmartDashboard.putData("Drivebase: Reset Encoders", new ResetDriveEncoders());
		SmartDashboard.putData("Shooter: Calibrate", new ShooterOneWayCalibrate());
		SmartDashboard.putData("AHRS: Reset Gyro", new ResetGyro());
		SmartDashboard.putData("Vision: Target Goal", new TargetGoal());
		SmartDashboard.putData("Vision: Calibrate", new VisionCalibrate());
		SmartDashboard.putData("Vision: AimBot", new AimBot());
		
		//Test Buttons
		SmartDashboard.putData("Test AutoTarget", new AutoTarget());
		SmartDashboard.putData("Test Motion Profiling", new DriveMotionProfiling(20, 0.1, 0.1, 3, true));
	}
}

