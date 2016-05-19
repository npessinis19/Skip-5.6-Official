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
	
	private int state = 0;
	
	/**
	 * @param Distance
	 * @param Acceleration
	 * @param Deceleration
	 * @param Cruise Velocity
	 * @param Period
	 * @param Reset Encoders
	 */
	public ShootMP() {
		requires(pultaCat);

		this.setInterruptible(true);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
	}

	private void armSetUp() {
		switch (state) { 
		//Down with the bourgeois (pultaCat)
		case 1: 
			pultaCat.startMP(pultaCat.downTrajectory);
			state = 2;
			break;
		//Check if execution of bourgeois is finished and latches
		case 2:
			if (pultaCat.isComplete() && pultaCat.resetBarIsLatched()) {
				pultaCat.cataLatch();
				state = 3;
			}
			break;
		//Rise of the bourgeois
		case 3:
			pultaCat.startMP(pultaCat.upTrajectory);
			state = 4;
			break;
		//Check if execution of profile is finished
		case 4:
			if (pultaCat.isComplete() && pultaCat.resetBarIsClear()) {
				end();
			}
		default:
			System.out.println("You suck and need to pick 1");
			break;
		}
	}
	
	@Override
 	protected void initialize() {
		state = 1;
		System.out.println("Starting pultaCat!");
	}

	@Override
	protected void execute() {
		armSetUp();
		pultaCat.publishValues();
		SmartDashboard.putNumber("ShooterSetpoint", pultaCat.getResetAngle());
		SmartDashboard.putNumber("Shooter state: ", state);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		state = 0;
		System.out.println("Finished with pultaCat!");
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}

}
