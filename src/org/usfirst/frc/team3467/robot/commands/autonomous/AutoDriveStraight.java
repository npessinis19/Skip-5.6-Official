package org.usfirst.frc.team3467.robot.commands.autonomous;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.ResetDriveEncoders;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterPrepare;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *  Drive straight ahead for specified distance
 */
public class AutoDriveStraight extends CommandGroup {
    
    public  AutoDriveStraight() {
    	addSequential(new DriveStraight(100));
		addSequential(new Roller_Actuate(true));
		addSequential(new ResetDriveEncoders());
    	addSequential(new DriveStraight(9900));
    }
}
