package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;


public class AimBot extends CommandBase {
	
		private BuildTrajectory trajectory;
		private MP_CANTalons leftmp_drive, rightmp_drive;
		private int aimState = 1;
		
		private static boolean debugging = true;
		
		
		public AimBot() {
			requires(driveBase);
			buildControllers();
			
			this.setInterruptible(false);
			
			setTimeout(20);
		}
		
		
		public void buildControllers() {
			leftmp_drive = new MP_CANTalons("Left Drive", driveBase.getLeftTalon(), false);
			rightmp_drive = new MP_CANTalons("Right Drive", driveBase.getRightTalon(), false);
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
				Thread.sleep(2000);
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
		}
		
		public void autoAim() {
			switch (aimState) {
			
			//State one, search for a good image
			case 1: if (!grip.isGoodImage()) {
						grip.createImage();
					}
					else {
						aimState = 2;
					}
				break;
				
			//Calculate target Data
			case 2:
					grip.calculateTargetData();
					trajectory = new BuildTrajectory((int) grip.getChangeinAngle(), 3, 3, 10, 10);
					aimState = 3;
				break;
			//Move to correct position
			case 3:
					startMP();
					aimState = 4;
				break;
			//Check if we're on Target (if not, go to aimState 1)
			case 4: 
					if (!grip.imageOnTarget) {
						aimState = 1;
					}
					else {
						System.out.println("AutokAim on Target");
						end();
					}
				break;
			}
		}
		
		
		protected void initialize() {
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			driveBase.resetEncoders();
			
			driveBase.getLeftTalon().reverseOutput(true);
			driveBase.getRightTalon().reverseOutput(true);
			
			ahrs.gyroReset();
			
			startMP();
		}

		protected void execute() {
			ahrs.reportGyroValues();
			driveBase.reportEncoders();
			
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			leftmp_drive.upDateMotionProfileStatus();
			//leftmp_drive.testExecuteOutput();
			
			leftmp_drive.enableMotionProfiling();
			rightmp_drive.enableMotionProfiling();
			
			autoAim();
			
			//leftmp_drive.stallOnLastPoint();
			//rightmp_drive.stallOnLastPoint();
			
			SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
		}

		protected boolean isFinished() {
			return false;
		}

		protected void end() {
			leftmp_drive.clearMotionProfileTrajectories();
			rightmp_drive.clearMotionProfileTrajectories();
			
			resetMP();
		}

		protected void interrupted() {
			end();
		}
	}
