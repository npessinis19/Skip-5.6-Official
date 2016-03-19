package org.usfirst.frc.team3467.robot.subsystems.compressor.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

public class Compressor_Update extends CommandBase {

	private int counter;
	
	public Compressor_Update() {
		requires(pneumatics);
		this.setInterruptible(true);
	}
	
	protected void initialize() {
		counter = 0;
	}

	protected void execute() {
		if (counter < 50) {
			counter++;
		}
		else {
			pneumatics.reportPressure();
			counter = 0;
		}
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		
	}

	protected void interrupted() {
		end();
	}
}
