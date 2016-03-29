package org.usfirst.frc.team3467.robot.commands.autonomous;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.AutoRotateToAngle;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.TargetGoal;
import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoTarget extends CommandGroup {
	
	public AutoTarget() {
		addSequential(new TargetGoal());
		addSequential(new AutoRotateToAngle(CommandBase.grip.getChangeinAngle()));
		addSequential(new TargetGoal());
		//addSequential(new DriveStraight(CommandBase.grip.changeinDistance));
		//addSequential(new TargetGoal());
		//addSequential(new AutoRotateToAngle(CommandBase.grip.changeinAngle));
		//addSequential(new DriveStraight(CommandBase.grip.changeinDistance));
	}

}
