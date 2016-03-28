package org.usfirst.frc.team3467.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.AutoRotateToAngle;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterPrepare;

public class AutoPortCullis extends CommandGroup {
	
	public AutoPortCullis() {
		
		addSequential(new Roller_Actuate(true));
		//addSequential(new ShooterPrepare());
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new Bar_actuate(UtilityBar.kOut));
		addSequential(new DriveStraight(3500));
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new DriveStraight(2000, 0.8));
	}
}
