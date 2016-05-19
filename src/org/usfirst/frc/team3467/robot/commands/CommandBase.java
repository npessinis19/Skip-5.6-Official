package org.usfirst.frc.team3467.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team3467.robot.OI;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.DriveBase;
import org.usfirst.frc.team3467.robot.subsystems.BallEject.BallEject;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.subsystems.NavX_MXP.MXP_AHRS;
import org.usfirst.frc.team3467.robot.subsystems.utilitybar.UtilityBar;
import org.usfirst.frc.team3467.robot.subsystems.Intake.Intake;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.Shooter;
import org.usfirst.frc.team3467.robot.subsystems.Vision.Flashlight;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;
import org.usfirst.frc.team3467.robot.subsystems.Vision.Video;
import org.usfirst.frc.team3467.robot.subsystems.compressor.Pneumatics;

public abstract class CommandBase extends Command {
	
		//Create universal examples of subsystems
	public static CommandBase commandBase;
	public static OI oi;
	public static Pneumatics pneumatics;
	public static MXP_AHRS ahrs;
	public static DriveBase driveBase;
	public static Brownout brownout;
	public static UtilityBar utilitybar;
	public static Shooter pultaCat;
	public static Intake intake;
	public static BallEject ballEject;
	
	//Vision Classes
	public static Flashlight light;
	public static GRIP grip;
	public static Video video;
	
		//Create vector with subsystems as elements for global subsystem commands
	public static Vector <Subsystem> subsystemList;
	
	
	public static void init() {
		System.out.println("Command Base Initialized");
		
		//Make instance of vector known as subsystemList
		subsystemList = new Vector<Subsystem>();
		
		//Add instances of subsystems
		pneumatics = Pneumatics.getInstance();
		subsystemList.addElement(pneumatics);		
		brownout = Brownout.getInstance();
		subsystemList.addElement(brownout);

		// Get instances of subsystems
		driveBase = new DriveBase();
		subsystemList.addElement(driveBase);
		ahrs = new MXP_AHRS();
		subsystemList.addElement(ahrs);
		utilitybar = new UtilityBar();
		subsystemList.addElement(utilitybar);
		pultaCat = new Shooter();
		subsystemList.addElement(pultaCat);
		intake = new Intake();
		subsystemList.addElement(intake);
		light = new Flashlight();
		subsystemList.addElement(light);
		ballEject = new BallEject();
		subsystemList.addElement(ballEject);
		
		//Non-Subsystem Classes
		//video = new Video();
		//grip = new GRIP();
		oi = new OI();

		//Initial Commands
		oi.BindCommands();	
	}
	
	public CommandBase() {
		super();
		commandBase = this;
	}
	public CommandBase (String name) {
		super(name);
	}
}