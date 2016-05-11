package org.usfirst.frc.team3467.robot.subsystems.BallEject;

import org.usfirst.frc.team3467.robot.;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class EjectCommand extends CommandBase{

	private Value INorOUT;
	
	public EjectCommand(Value inORout) {
	 requires(BallEject);
	 INorOUT = inORout;
	 setTimeout(1.0);
	}
	
	public EjectCommand() {
		requires(BallEject);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}

}
