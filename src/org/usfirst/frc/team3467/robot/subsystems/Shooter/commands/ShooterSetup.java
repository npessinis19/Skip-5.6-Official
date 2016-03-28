package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

public class ShooterSetup extends CommandBase {

	boolean isClearing = false;
	boolean isLatching = false;
	boolean isBrowned = false;
	
	public ShooterSetup() {
		requires(pultaCat);
		requires(pneumatics);
		setTimeout(2);
	}
	
	//Called just before this command runs for the first time 
	protected void initialize() {
		pultaCat.cataLatch();
		pneumatics.compressorStop();
		pultaCat.initPIDMode();
		pultaCat.latch();
		isLatching = true;
		
		System.out.println("Shooter Setup: Latching");		
	}

	protected void execute() {
		
		// Only worry about brownout while latching
		if (isLatching == true) {
			pultaCat.checkCurrent();
			if (pultaCat.checkBrownOut()) {
				isBrowned = true;

				System.out.println("Shooter Setup: Browned Out!");
			}
			
			if (pultaCat.resetBarIsLatched() || isBrowned) {
				pultaCat.clear();
				isLatching = false;
				isClearing = true;
				
				System.out.println("Shooter Setup: Clearing");
			}
		}
		
	}

	protected boolean isFinished() {
		// Only timeout on the latching portion of setup
		if ((isLatching && isTimedOut()) || (isClearing && pultaCat.resetBarIsClear())) {
			System.out.println("Shooter Setup: Finished");
			return(true);
		}
		else
			return(false);
	}

	protected void end() {
		pultaCat.cataStop();
		pneumatics.compressorStart();
		pultaCat.initManualMode();
		System.out.println("Shooter is setup");
	}

	protected void interrupted() {
		end();
	}
}
