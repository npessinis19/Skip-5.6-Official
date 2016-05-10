package org.usfirst.frc.team3467.robot.commands.autonomous;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.AutoIntake;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.Shoot;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.AimBot;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ChevalAndShoot extends CommandGroup {
	
	public ChevalAndShoot() {
	addSequential(new Bar_actuate(UtilityBar.kOut));
	addSequential(new Bar_actuate(UtilityBar.kIn));
	addSequential(new DriveStraight(1000, 0.5));
	addSequential(new DriveStraight(2350, 0.5));
	addSequential(new DriveStraight(-285, 0.2));
	addSequential(new Bar_actuate(UtilityBar.kOut));
	//addSequential(new DriveStraight(100, 0.1));
	addSequential(new DriveStraight(6400, 0.5));
	addSequential(new AutoIntake(true), 2.0);
	addSequential(new AimBot());
	addSequential(new Shoot());
	}
}
