package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootMP extends CommandBase {
	
	private static boolean debugging = true;
	private static double TOLERANCE = 0.5;
	private double m_angle = 0.0;
	private boolean m_reset;
	
	/**
	 * @param Distance
	 * @param Acceleration
	 * @param Deceleration
	 * @param Cruise Velocity
	 * @param Period
	 * @param Reset Encoders
	 */
	public ShootMP(int xnet, double accel, double decel, double cruise, double step, boolean reset) {
		requires (shooter);

		
		m_reset = reset;
		this.setInterruptible(true);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
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
