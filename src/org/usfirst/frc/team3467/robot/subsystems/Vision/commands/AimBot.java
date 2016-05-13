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
				leftmp_drive.processMotionProfileBuffer();
				rightmp_drive.processMotionProfileBuffer();
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
		
		public void startMP() {
			System.out.println("Start Auto Aim Motion Profiling");
			
			resetMP();
			
			leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
			rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
			
			//leftmp_drive.testProfile();
			//rightmp_drive.testProfile();
			
			leftmp_drive.changeMotionControlFramePeriod(20);
			rightmp_drive.changeMotionControlFramePeriod(20);
			
			notifier.startPeriodic(.005);
					
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			if (debugging) {
				leftmp_drive.upDateMotionProfileStatus();
				rightmp_drive.upDateMotionProfileStatus();
			
				SmartDashboard.putNumber("Init Left Botttom Buffer", leftmp_drive.BottomBufferCount());
				SmartDashboard.putNumber("Init Right Bottom Buffer", rightmp_drive.BottomBufferCount());
			}
			
			leftmp_drive.enableMotionProfiling();
			rightmp_drive.enableMotionProfiling();
		}
		
		public void resetMP() { 
			leftmp_drive.clearMotionProfileTrajectories();
			rightmp_drive.clearMotionProfileTrajectories();
			
			leftmp_drive.disableMotionProfiling();
			rightmp_drive.disableMotionProfiling();
			
			System.out.println("MP Reset");
		}

		public void publishValues() {
			if (debugging) {
				leftmp_drive.upDateMotionProfileStatus();
				rightmp_drive.upDateMotionProfileStatus();
				
				SmartDashboard.putNumber("Left Top Buffer", leftmp_drive.TopbufferCount());
				SmartDashboard.putNumber("Right Top Buffer", rightmp_drive.TopbufferCount());
				
				SmartDashboard.putNumber("Left Bottom Buffer", leftmp_drive.BottomBufferCount());
				SmartDashboard.putNumber("Right Bottom Buffer", rightmp_drive.BottomBufferCount());
				
				SmartDashboard.putBoolean("Left Is Underrun", leftmp_drive.isUnderrun());
				SmartDashboard.putBoolean("Right Is Underrun", rightmp_drive.isUnderrun());
				
				SmartDashboard.putBoolean("Left Has Underrun", leftmp_drive.hasUnderrun());
				SmartDashboard.putBoolean("Right Has Underrrun", rightmp_drive.hasUnderrun());
				
				System.out.println("Left Active Point " + leftmp_drive.getActivePoint().position + " Time " + leftmp_drive.getActivePoint().timeDurMs);
				System.out.println("Right Active Point" + rightmp_drive.getActivePoint().position + " Time " + rightmp_drive.getActivePoint().timeDurMs);
			}
			SmartDashboard.putNumber("Vision: AimState", aimState);
		}
		
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
					
						aimState = 1;

					if (trajectory != null) {
						System.out.println("AimBot Calculated");
						
						aimState = 3;
					}
					else {
						System.out.println("Failed to make trajectory");
						
						aimState = 1;
					}
				break;
			//State 3: Move to correct position
			case 3:
					light.lightOff();
					startMP();
					
					leftmp_drive.enableMotionProfiling();
					rightmp_drive.enableMotionProfiling();
					
					System.out.println("AimBot moving");
					
					aimState = 4;
				break;
			//State 4, wait for move to finish	
			case 4:
					leftmp_drive.enableMotionProfiling();
					rightmp_drive.enableMotionProfiling();
				
					if (leftmp_drive.isComplete()) {
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
			
			leftmp_drive.clearMotionProfileTrajectories();
			rightmp_drive.clearMotionProfileTrajectories();
			
			aimState = 1;
			
			ahrs.gyroReset();
		}

		protected void execute() {
			ahrs.reportGyroValues();
			driveBase.reportEncoders();
			
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			
			publishValues();
			
			autoAim();
			
			//leftmp_drive.stallOnLastPoint();
			//rightmp_drive.stallOnLastPoint();
			
			SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
		}

		protected boolean isFinished() {
			return finished || isTimedOut();
		}

		protected void end() {
			leftmp_drive.clearMotionProfileTrajectories();
			rightmp_drive.clearMotionProfileTrajectories();
			
			resetMP();
			
			driveBase.initDrive();
			driveBase.driveArcade(0, 0, false);
		}

		protected void interrupted() {
			resetMP();
		}
	}
