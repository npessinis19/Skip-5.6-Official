package org.usfirst.frc.team3467.robot.subsystems.Shooter.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootMP extends CommandBase {
	
	private int state = 0;
	
	public ShootMP() {
		requires(pultaCat);
		
		setTimeout(2);

		this.setInterruptible(true);
		
		SmartDashboard.putString("TestProfiling Mode", "position");
	}

	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			pultaCat.processMotionProfileBuffer();
		}
		
	}
	
	Notifier notify = new Notifier(new PeriodicRunnable());
	
	private void armSetUp() {
		switch (state) { 
		//Down with the bourgeois (pultaCat)
		case 1: 
			pultaCat.startMPwithAddition(pultaCat.shooterTrajectory, true, 491);
			notify.startPeriodic(0.005);
			state = 2;
			break;
		//Check if execution of bourgeois is finished and latches
		/*case 2:
			if (pultaCat.isComplete() || pultaCat.resetBarIsLatched()) {
				pultaCat.resetMP();
				pultaCat.cataLatch();
				state = 3;
			}
			break;
		//Rise of the bourgeois
		case 3:
			pultaCat.startMPwithAddition(pultaCat.shooterTrajectory, false, 62);
			state = 4;
			break;
		//Check if execution of profile is finished
		case 4:
			if (pultaCat.isComplete() || pultaCat.resetBarIsClear()) {
				end();
			}
			*/
		default:
			System.out.println("You suck and need to pick 1");
			break;
		}
	}
	
	
 	protected void initialize() {
		state = 1;
		System.out.println("Starting pultaCat!");
	}

	protected void execute() {
		armSetUp();
		pultaCat.publishValues();
		SmartDashboard.putNumber("ShooterSetpoint", pultaCat.getResetAngle());
		SmartDashboard.putNumber("Shooter state: ", state);
	}

	protected boolean isFinished() {
		return isTimedOut();
	}

	protected void end() {
		pultaCat.resetMP();
		notify.stop();
		state = 0;
		System.out.println("Finished with pultaCat!");
	}

	protected void interrupted() {
	}
}
