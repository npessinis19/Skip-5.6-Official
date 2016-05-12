package org.usfirst.frc.team3467.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.TargetGoal;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.*;

public class AutoCheval extends CommandGroup {
	
	/*
	 * Autonomous for cheval de Frieze
	 * 		-Utilitybar in
	 * 		-Moves Forward
	 * 		-Utilitybar out
	 *		-Moves Forward Again
	 *		-Finger in
	 *		-Turns and shoots
	 *		-Stops
	 */
	
	public AutoCheval() {
		addSequential(new Bar_actuate(UtilityBar.kOut));
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new DriveStraight(1000, 0.2));
		addSequential(new DriveStraight(2350, 0.5));
		addSequential(new DriveStraight(-250, 0.2));
		addSequential(new Bar_actuate(UtilityBar.kOut));
		//addSequential(new DriveStraight(100, 0.1));
		addSequential(new DriveStraight(6400, 0.5));
	}
}
