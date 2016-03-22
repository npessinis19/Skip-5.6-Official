package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

public class ShooterSetup extends CommandBase {

	boolean isClear = false;
	
	public ShooterSetup() {
		requires(pultaCat);
	}
	
	
	//Called just before this command runs for the first time 
	protected void initialize() {
		pultaCat.cataLatch();
		pultaCat.initPIDMode();
		pultaCat.latch();
		
		if (pultaCat.checkLatchLimit() == true || pultaCat.onTarget()) {
			pultaCat.clear();
			isClear = true;
			System.out.println("Shooter is setup");
		}
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isClear;
	}

	@Override
	protected void end() {
		pultaCat.cataStop();
		pultaCat.initManualMode();
	}

	protected void interrupted() {
		end();
	}
	
	

}
