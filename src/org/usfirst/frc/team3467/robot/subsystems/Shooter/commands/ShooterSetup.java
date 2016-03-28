package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

public class ShooterSetup extends CommandBase {

	boolean isClear;
	boolean latching;
	
	public ShooterSetup() {
		requires(pultaCat);
		setTimeout(2);
	}
	
	//Called just before this command runs for the first time 
	protected void initialize() {
		pultaCat.cataLatch();
		pultaCat.initPIDMode();
		pultaCat.latch();
		
		System.out.println("Shooter Setup: Latching");
		
		isClear = false;
		latching = true;
	}

	protected void execute() {
		
		if (pultaCat.checkBrownOut()) {
			System.out.println("Shooter Browned out");
			end();
		}
		
		if (pultaCat.checkLatchLimit() || pultaCat.resetBarIsLatched()) {
			pultaCat.clear();
			isClear = true;
			
			System.out.println("Shooter Setup: Clearing");
		}
	}

	protected boolean isFinished() {
		return (isClear && pultaCat.resetBarIsClear()) || isTimedOut();
	}

	protected void end() {
		pultaCat.cataStop();
		pultaCat.initManualMode();
		System.out.println("Shooter is setup");
	}

	protected void interrupted() {
		end();
	}
}
