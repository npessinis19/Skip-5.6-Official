package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;


public class AimBot extends CommandBase {

		private MP_CANTalons leftmp_drive, rightmp_drive;
		private BuildTrajectory trajectory;
		
		private int aimState = 1;
		private boolean finished = false;
		private static boolean debugging = true;
		private static final double TOLERANCE = 0.1;
		private double m_angle;
			
		
		public AimBot() {
			requires(driveBase);
			this.setInterruptible(true);
			setTimeout(20);
			
			buildControllers();
		}

		public void buildControllers() {
			leftmp_drive = new MP_CANTalons("Left Drive", driveBase.getLeftTalon(), false);
			rightmp_drive = new MP_CANTalons("Right Drive", driveBase.getRightTalon(), false);
		}
		
		class PeriodicRunnable implements java.lang.Runnable {
			public void run() {
				leftmp_drive.processMotionProfileBuffer();
				rightmp_drive.processMotionProfileBuffer();
			}
		}
		
		Notifier notifier = new Notifier(new PeriodicRunnable());
		
		
		public void startMP() {
			SmartDashboard.putString("TestProfiling Message", "startMP Called");
			
			resetMP();
			
			System.out.println("Starting Motion Profiling");
			
			leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
			rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		
			leftmp_drive.changeMotionControlFramePeriod(5);
			rightmp_drive.changeMotionControlFramePeriod(5);

			notifier.startPeriodic(0.005);
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			leftmp_drive.upDateMotionProfileStatus();
			rightmp_drive.upDateMotionProfileStatus();
			
			SmartDashboard.putNumber("Init Left Bottom Buffer", leftmp_drive.BottomBufferCount());
			SmartDashboard.putNumber("Init Right Bottom Buffer", rightmp_drive.BottomBufferCount());
			
			leftmp_drive.enableMotionProfiling();
			rightmp_drive.enableMotionProfiling();
		}
		
		public void resetMP() {
			leftmp_drive.upDateMotionProfileStatus();
			rightmp_drive.upDateMotionProfileStatus();
			int increment = 0;
			
			while (leftmp_drive.BottomBufferCount() > 0 && rightmp_drive.BottomBufferCount() > 0 || increment > 400) {
				leftmp_drive.clearMotionProfileTrajectories();
				rightmp_drive.clearMotionProfileTrajectories();

				increment++;
				
				leftmp_drive.upDateMotionProfileStatus();
				rightmp_drive.upDateMotionProfileStatus();
				System.out.println("Incrmenet " + leftmp_drive.BottomBufferCount());
				if (increment == 400) {
					break;
				}
			}
			
			
			
			
			leftmp_drive.disableMotionProfiling();
			rightmp_drive.disableMotionProfiling();
			
			notifier.stop();
			
			System.out.println("MP Reset");
		}
		
		public void publishValues() {
			leftmp_drive.upDateMotionProfileStatus();
			rightmp_drive.upDateMotionProfileStatus();
			
			SmartDashboard.putNumber("Left Top Buffer",leftmp_drive.topBuffercount());
			SmartDashboard.putNumber("Right Top Buffer", rightmp_drive.topBuffercount());
			
			SmartDashboard.putNumber("Left Bottom Buffer", leftmp_drive.BottomBufferCount());
			SmartDashboard.putNumber("Right Bottom Buffer", rightmp_drive.BottomBufferCount());
			
			SmartDashboard.putBoolean("Left Is Underrun", leftmp_drive.isUnderrun());
			SmartDashboard.putBoolean("Right Is Underrun", rightmp_drive.isUnderrun());
			
			SmartDashboard.putBoolean("Left Has Underrun", leftmp_drive.hasUnderrun());
			SmartDashboard.putBoolean("Right Has Underrun", rightmp_drive.hasUnderrun());
		
			System.out.println("Active Point" + rightmp_drive.getActivePoint().position);
			
			SmartDashboard.putString("Talon Mode", driveBase.getTalonControlMode());
		}
		
		public void TalonOutput() {
			if (m_angle < 0) {
				driveBase.getLeftTalon().reverseOutput(false);
				driveBase.getRightTalon().reverseOutput(false);
			}
			else if (m_angle > 0){
				driveBase.getLeftTalon().reverseOutput(true);
				driveBase.getRightTalon().reverseOutput(true);
			}
			else {
				driveBase.getLeftTalon().reverseOutput(true);
				driveBase.getRightTalon().reverseOutput(false);
			}
		}
		
		public void autoAim() {
			switch (aimState) {
			//State 1, search for a good image
			case 1: 
					m_angle = camera.convert();
					
					System.out.println("AimBot Found Good Image");
					
					aimState = 2;
				break;
			//State 2, Build Trajectory
			case 2:
						System.out.println("Aimbot found Change in Angle" + m_angle);
						
						trajectory = new BuildTrajectory((int) ((m_angle) * 14), 0.02, 0.02 , 4, 10);

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
					
					startMP();
					
					System.out.println("AimBot moving");
					
					aimState = 4;
				break;
			//State 4, wait for move to finish	
			case 4:
					if (leftmp_drive.isComplete() || rightmp_drive.isComplete()) {
						finished = true;
						System.out.println("AimBot moved");
					}
					else {
						finished = false;
						}
				break;
			//State 5, check if we're on Target (if not, go to aimState 1)
			case 5: 
					double testAngle = camera.convert();
					
					if (Math.abs(testAngle) >= TOLERANCE) {
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
				break;
			}
		}
		
		public void removeSlack() {
			if (m_angle < 3 && m_angle > 0) {
				driveBase.setControlMode(TalonControlMode.PercentVbus);
				
				driveBase.getLeftTalon().set(0.03);
				driveBase.getRightTalon().set(0.03);
			}
			else if (m_angle > -3 && m_angle < 0)  {
				driveBase.setControlMode(TalonControlMode.PercentVbus);
			
				driveBase.getLeftTalon().set(-0.03);
				driveBase.getRightTalon().set(-0.03);
			}
		}
		
		protected void initialize() {
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			driveBase.resetEncoders();
				
			driveBase.getLeftTalon().reverseOutput(true);
			driveBase.getRightTalon().reverseOutput(true);
			
			removeSlack();
			
			driveBase.getLeftTalon().clearIAccum();
			driveBase.getRightTalon().clearIAccum();
			
			leftmp_drive.clearMotionProfileTrajectories();
			rightmp_drive.clearMotionProfileTrajectories();
			
			driveBase.setTalonBrakes(false);
			driveBase.setSlaveMode(true);
			
			ahrs.gyroReset();
			
			aimState = 1;
			finished = false;
		}
		
		protected void execute() {
			autoAim();
			
			ahrs.reportGyroValues();
			driveBase.reportEncoders();
			
			driveBase.setControlMode(TalonControlMode.MotionProfile);
			
			publishValues();
			
			SmartDashboard.putNumber("Auto Aim Angle", m_angle);
			SmartDashboard.putNumber("Aim State", aimState);
			
			leftmp_drive.enableMotionProfiling();
			rightmp_drive.enableMotionProfiling();
		}
		
		protected boolean isFinished() {
			double error = Math.abs(m_angle - ahrs.getGyroAngle());
			boolean complete = (leftmp_drive.isComplete() || rightmp_drive.isComplete());
			
			return isTimedOut();
			//return error <= TOLERANCE || isTimedOut();
		}

		protected void end() {
			resetMP();
			
			if (Math.abs(camera.convert()) < TOLERANCE) {
				light.lightOn();
			}
			
			driveBase.initDrive();
			//light.lightOn();
		}
		
		//Separate Thread that Processes points to Bottom Buffer


	
		protected void interrupted() {
			end();
		}
	}
