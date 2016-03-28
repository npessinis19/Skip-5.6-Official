package org.usfirst.frc.team3467.robot.commands.autonomous;

import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterPrepare;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoGetReady extends CommandGroup {

	public AutoGetReady() {
		//addSequential(new Roller_Actuate(true));
		//addSequential(new ShooterPrepare());
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new Bar_actuate(UtilityBar.kOut));
	}
	
}
