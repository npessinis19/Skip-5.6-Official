package org.usfirst.frc.team3467.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;

public class AutoPortCullis extends CommandGroup {
	
	public AutoPortCullis() {
		
		addSequential(new Roller_Actuate(true));
		//addSequential(new ShooterPrepare());
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new Bar_actuate(UtilityBar.kOut));
		addSequential(new DriveStraight(5500));
	}
}
