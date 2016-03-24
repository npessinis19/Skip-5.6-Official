package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

public class ShooterSetup extends CommandBase {

	boolean isClear;
	boolean latching;
	
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
		
		System.out.println("Shooter Setup: Latching");
		
		isClear = false;
		latching = true;
	}

	protected void execute() {
		
		pultaCat.callbackAlert(CommandBase.brownout.getLevel());
		pultaCat.checkCurrent();
		
		if (pultaCat.checkBrownOut()) {
			System.out.println("Shooter Browned Out");
			end();
		}
		
		if (pultaCat.checkLatchLimit() || pultaCat.resetBarIsLatched()) {
			pultaCat.clear();
			isClear = true;
			
			System.out.println("Shooter Setup: Clearing");
		}
	}

	protected boolean isFinished() {
		// Only timeout on the latching portion of setup
		if ((!isClear && isTimedOut()) || (isClear && pultaCat.resetBarIsClear())) {
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
