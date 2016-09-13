package org.usfirst.frc.team3467.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveMotionProfiling;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.AutoIntake;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.Shoot;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterOneWayCalibrate;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.AimBot;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.LightSwitch;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;

public class AutoLowBar extends CommandGroup {
	
	public AutoLowBar() {
		//addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new Bar_actuate(UtilityBar.kOut));
		//addSequential(new JustDriveFor5(0.5));
		addSequential(new DriveStraight(50));
		addSequential(new Roller_Actuate(true));
		//addSequential(new ShooterPrepare());
		//addSequential(new JustDriveFor5(6));
		addSequential(new DriveStraight(7950));
		addSequential(new Roller_Actuate(false));
		addSequential(new DriveStraight(2000, true, 0.02));
		addSequential(new DriveMotionProfiling(42, 0.1, 0.1, 3, true));
		addSequential(new AimBot());
		addSequential(new AimBot());
		addSequential(new AimBot());
		//addSequential(new AimBot());
		//addSequential(new AutoIntake(true), 2.0);
		addSequential(new Shoot());
		//addSequential(new LightSwitch(true));
		//addSequential(new ShooterOneWayCalibrate());
		//addSequential(new Shoot());
	}
}
