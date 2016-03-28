package org.usfirst.frc.team3467.robot.commands.autonomous;

import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.DriveStraight;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.ResetDriveEncoders;
import org.usfirst.frc.team3467.robot.subsystems.Intake.commands.Roller_Actuate;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterPrepare;
import org.usfirst.frc.team3467.robot.subsystems.Vision.commands.LightSwitch;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Bar_actuate;
//import org.usfirst.frc.team3467.robot.subsystems.utilitybar.commands.Finger_actuate;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *  Drive straight ahead for specified distance
 */
/* Auto Drive Straight (Rough, Ramps, Molt, Rock Wall)
 * Move Forward
 * Lower Intake
 * Drive some more
 * Utility Bar and finger prepare
 * Turn on LightSwitch
 */

public class AutoDriveStraight extends CommandGroup {
    
    public  AutoDriveStraight() {
    	addSequential(new DriveStraight(50));
		addSequential(new Roller_Actuate(true));
		addSequential(new Bar_actuate(UtilityBar.kIn));
		addSequential(new ResetDriveEncoders());
    	addSequential(new DriveStraight(9950));
    	//addSequential(new Bar_actuate(UtilityBar.kIn));
    	addSequential(new Bar_actuate(UtilityBar.kOut));
    	addSequential(new LightSwitch(true));
    }
}
