package org.usfirst.frc.team3467.robot.subsystems.BallEject;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class EjectCommand extends CommandBase {

	private Value INorOUT;
	
	public EjectCommand(Value inORout) {
		requires(ballEject);
		INorOUT = inORout;
		setTimeout(1.0);
	}

	protected void initialize() {
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
	}

	protected void interrupted() {
	}
}
