package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;


public class AimBot extends CommandBase {
	
		private BuildTrajectory trajectory;
		
		private int aimState = 1;
		private boolean finished = false;
		private static boolean debugging = true;
		
		
		public AimBot() {
			requires(driveBase);
			this.setInterruptible(true);
			
			//setTimeout(3);
		}
		
		
		//Separate Thread that Processes points to Bottom Buffer
		class PeriodicRunnable implements java.lang.Runnable {
			public void run() { 
				driveBase.processMotionProfileBuffer();
			}
		}
		
		
		/*
		public DriveMotionProfiling(){
			leftmp_drive.changeMotionControlFramePeriod(1);
			//rightmp_drive.changeMotionControlFramePeriod(1);
			notifier.startPeriodic(.001);
		}
		*/
		
		Notifier notifier = new Notifier(new PeriodicRunnable());
		
		public void autoAim() {
			switch (aimState) {
			//State 1, search for a good image
			case 1: 
					grip.createImage();
					
					System.out.println("AimBot Found Good Image");
					
					aimState = 2;
				break;
			//State 2, Calculate target Data
			case 2:
						grip.calculateTargetData();
						grip.printData();
						
						System.out.println("Aimbot found Change in Angle" + grip.getChangeinAngle());
						
						trajectory = new BuildTrajectory((int) (grip.getChangeinAngle() - 1) * 14, 0.05, 0.05, 2, 10);

					if (trajectory != null) {
						System.out.println("AimBot Calculated");
						aimState = 3;
					}
					else {
						System.out.println("Failed to make trajectory: falling back to target search");
						aimState = 1;
					}
				break;
			//State 3: Move to correct position
			case 3:
					light.lightOff();
					
					driveBase.startMP(trajectory);
					
					System.out.println("AimBot moving");
					
					aimState = 4;
				break;
			//State 4, wait for move to finish	
			case 4:
				
					if (driveBase.isComplete()) {
						System.out.println("AimBot moved");
						aimState = 5;
					}
				break;
			//State 5, check if we're on Target (if not, go to aimState 1)
			case 5: 
					grip.createImage();
					grip.calculateTargetData();
					
					if (!grip.isOnTarget()) {
						aimState = 1;
					}
					else {
						aimState = 6;
						System.out.println("AutokAim on Target");
					}
				break;
			//State 6: Ends the command
			case 6:
					finished = true;
			}
		}
		
		
		protected void initialize() {
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			driveBase.resetEncoders();
			
			driveBase.getLeftTalon().reverseOutput(true);
			driveBase.getRightTalon().reverseOutput(true);
			
			driveBase.resetMP();
			
			aimState = 1;
			
			ahrs.gyroReset();
		}

		protected void execute() {
			ahrs.reportGyroValues();
			driveBase.reportEncoders();
			
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			
			//publishValues();
			
			autoAim();
			
			//leftmp_drive.stallOnLastPoint();
			//rightmp_drive.stallOnLastPoint();
			
			SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
		}

		protected boolean isFinished() {
			return finished || isTimedOut();
		}

		protected void end() {
		//	leftmp_drive.clearMotionProfileTrajectories();
			//rightmp_drive.clearMotionProfileTrajectories();
			
			//resetMP();
			
			driveBase.initDrive();
			driveBase.driveArcade(0, 0, false);
		}

		protected void interrupted() {
		//	resetMP();
		}
	}
