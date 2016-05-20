package org.usfirst.frc.team3467.robot.motion_profiling;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.MotionProfileStatus;
import edu.wpi.first.wpilibj.CANTalon.TrajectoryPoint;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;

import java.util.ArrayList;


public class MP_CANTalons {
	
	//Constructor Objects
	private String m_name;
	private CANTalon m_talon;
	//private Notifier m_notifier;
	
	//Debugging IO outputs
	private static DigitalOutput testWriteOutput;
	private static DigitalOutput testExecuteOutput;
	private static DigitalOutput testProcessOutput;
	
	//Important Variables
	private boolean m_debugging = false;
	public boolean testFlashOn = false;
	private boolean bufferOutOnce = false;
	public boolean stopProduction = false;
	
	//Motion Profile Status Object
	private MotionProfileStatus m_status;
	
	
	public MP_CANTalons(String name, CANTalon talon, boolean debugging) {
		this.m_name = name;
		this.m_talon = talon;
		this.m_debugging = debugging;
		
		m_status = new MotionProfileStatus();
		//m_notifier = new Notifier(new PeriodicRunable());
		
		if (m_debugging) {
			testWriteOutput = new DigitalOutput(1);
			testExecuteOutput = new DigitalOutput(2);
			testProcessOutput = new DigitalOutput(3);
		}
		
		m_talon.changeControlMode(TalonControlMode.MotionProfile);
		m_talon.getMotionProfileStatus(m_status);
		
		m_talon.clearMotionProfileTrajectories();
	}

	class PeriodicRunable implements java.lang.Runnable {
		public void run() {
			processMotionProfileBuffer();
		}
	}
	
	
	public void initSmartDashboard() {
		SmartDashboard.putBoolean(m_name + "isUnderrun ", m_status.isUnderrun);
		SmartDashboard.putBoolean(m_name + "activePointValid ", m_status.activePointValid);
		
		SmartDashboard.putNumber(m_name + "TopBufferCount", m_status.topBufferCnt);
		SmartDashboard.putNumber(m_name + "BottomBuffercount", m_status.btmBufferCnt);
	}
	
	
	//Direct Calls to the CANTalons
	public synchronized void clearMotionProfileTrajectories() {
		m_talon.clearMotionProfileTrajectories();
	}

	public synchronized void setPID(double KP, double KI, double KD) {
		m_talon.setPID(KP, KI, KD);
	}

	public synchronized void processMotionProfileBuffer() {
		m_talon.processMotionProfileBuffer();
		
		if (m_debugging) {
			testProcessOutput.set(true);
			
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			testProcessOutput.set(false);
		}
	}
		
	public synchronized void pushMotionProfilingTrajectory(TrajectoryPoint point) {
		m_talon.pushMotionProfileTrajectory(point);
	}
	
	public synchronized void enableMotionProfiling() {
		m_talon.set(1);
		//System.out.println("Motion Profiling Enabled");
	}

	public synchronized void disableMotionProfiling() {
		m_talon.set(0);
		//System.out.println("Motion Profiling Disabled");
	}
	
	public synchronized void holdMotionProfiling() {
		m_talon.set(2);
	}
	
	public synchronized void changeMotionControlFramePeriod(int dur){
		m_talon.changeMotionControlFramePeriod(dur);
	}


	//Access the Motion Profiling Status object
	public synchronized CANTalon.MotionProfileStatus getStatus() {
		return m_status;
	}
	
	
	//Update the current Motion Profile Status object
	public synchronized void upDateMotionProfileStatus() {
		m_talon.getMotionProfileStatus(m_status);
	}

	
	//Retrieve Values from the Motion Profile Status Object
	public synchronized boolean isUnderrun() {
		return m_status.isUnderrun;
	}
	
	public synchronized boolean hasUnderrun() {
		return m_status.hasUnderrun;
	}
	
	public synchronized CANTalon.TrajectoryPoint getActivePoint() {
		return m_status.activePoint;
	}
	
	public synchronized int topBuffercount() {
		return m_status.topBufferCnt;
	}
	
	public synchronized int BottomBufferCount() {
		return m_status.btmBufferCnt;
	}
	
	public synchronized boolean isComplete() {
		boolean finished = false;

		upDateMotionProfileStatus();
		
		if(m_status.activePoint.isLastPoint) {
			finished = true;
		}
		return finished;
	}

	
	//Retrieve Values from CANTalons
	public synchronized int TopbufferCount() {
		return m_talon.getMotionProfileTopLevelBufferCount();
	}
		
	public synchronized boolean isTopBufferFull() {
		return m_talon.isMotionProfileTopLevelBufferFull();
	}
	
	
	
	public synchronized void testExecuteOutput() {
		if (m_debugging) {
		if (m_status.activePointValid == true) {
			testExecuteOutput.set(true);
		}
		else {
			testExecuteOutput.set(false);
			}
		}
	}
	
	
	/**
	 * Add constant number to trajectory points
	 * @param profile array list of trajectory data
	 * @param totalCount total number of trajectory points
	 * @param invert multiply points by -1
	 * @param addition add a number to trajectory points
	 */
	public void startFillingPlusSome(ArrayList <double[]> profile, int totalCount, boolean invert, int addition) {
		//Create an empty point
		CANTalon.TrajectoryPoint flag = new TrajectoryPoint();
		
		//Check if in Underrun Condition
		if (m_status.isUnderrun); System.out.println("Motion Profiling Is Underrun");
		
		//Clear isUnderrun Flag
		m_talon.clearMotionProfileHasUnderrun();
		
		for (int i = 0; i < totalCount; i++) {
				if (invert){
					flag.position = profile.get(i)[1] * -1 + addition;
					flag.timeDurMs = (int) profile.get(i)[0];
				}		
				else {
					flag.position = profile.get(i)[1] + addition;
					flag.timeDurMs = (int) profile.get(i)[0];
				}
				
		/*if (m_debugging); testWriteOutput.set(true);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
			*/
			
			//Use PID Slot 0 for Motion Profiling
			flag.profileSlotSelect = 0;
		
			System.out.println("Profile Points " + flag.position + " Time " + flag.timeDurMs + " Index " + i);
			
			//Only use velocities?
			flag.velocityOnly = false;
			flag.zeroPos = false;
		
			//Checks
			if (i == 0) {
				flag.zeroPos = false;
				flag.isLastPoint = false;
			}
			if ((i + 1) == totalCount); flag.isLastPoint = true;

			m_talon.pushMotionProfileTrajectory(flag);
			
			//System.out.println("Successful Push of Profile Point " + i /*m_talon.pushMotionProfileTrajectory(m_flag)*/);
			//testWriteOutput.set(false);
		}
	}
	
	/**
	 * startFilling generates a motion profile trajectory based on profile data in an ArrayList
	 * 
	 * @param Profile In the form of an Array List
	 * @param totalCount Length of profile
	 * @param invert Change the sign of all profile points
	 */
	public void startFilling(ArrayList <double[]> Profile, int totalCount, boolean invert) {

		//Create an empty point
		CANTalon.TrajectoryPoint flag = new TrajectoryPoint();
		
		//Check if in Underrun Condition
		if (m_status.isUnderrun); System.out.println("Motion Profiling Is Underrun");
		
		//Clear isUnderrun Flag
		m_talon.clearMotionProfileHasUnderrun();
		
		for (int i = 0; i < totalCount; i++) {
			if (invert) {
				flag.position = Profile.get(i)[1] * -1;
				flag.timeDurMs = (int) Profile.get(i)[0];
			}
			else {
				flag.position = Profile.get(i)[1];
				flag.timeDurMs = (int) Profile.get(i)[0];
			}
			
		/*if (m_debugging); testWriteOutput.set(true);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
			*/
			
			//Use PID Slot 0 for Motion Profiling
			flag.profileSlotSelect = 0;
		
			System.out.println("Profile Points " + flag.position + " Time " + flag.timeDurMs + " Index " + i);
			
			//Only use velocities?
			flag.velocityOnly = false;
			flag.zeroPos = false;
		
			//Checks
			if (i == 0) {
				flag.zeroPos = false;
				flag.isLastPoint = false;
			}
			if ((i + 1) == totalCount); flag.isLastPoint = true;

			m_talon.pushMotionProfileTrajectory(flag);
			
			//System.out.println("Successful Push of Profile Point " + i /*m_talon.pushMotionProfileTrajectory(m_flag)*/);
			//testWriteOutput.set(false);
		}
	}
	
	/**
	 * spliceTrajectory
	 * 		holds the current Motion Profile point,
	 * 		clears the old trajectory from both buffers,
	 * 		updates the Motion Profile Status,
	 * 		starts the motion profile executer and separate thread motion profile processor,
	 * 		and generates a new trajectory so long as the executer is not underrun,
	 * 		and production has NOT been stopped manually
	 * 
	 * @param profile In the form of an ArrayList
	 * @param totalCount Length of the profile
	 * @param invert change the signs of all profile points
	 */
	public void spliceTrajectory(ArrayList <double[]> profile, int totalCount, boolean invert) {
		//Preparation for new Profile
		holdMotionProfiling();
		clearMotionProfileTrajectories();
		upDateMotionProfileStatus();
		TrajectoryPoint flag = new TrajectoryPoint();
		
		//Check if Underrun
		if (m_status.isUnderrun); System.out.println("Motion Profiling is Underrun");
		m_talon.clearMotionProfileHasUnderrun();
		
		//Start executer and external thread processer
		enableMotionProfiling();
		//m_notifier.startPeriodic(0.005);
		
		for (int i = 0; i < totalCount && !stopProduction; i++) {
			if (invert) {
				flag.position = profile.get(i)[1] * -1;
				flag.timeDurMs = (int) profile.get(i)[0];
			}
			else {
				flag.position = profile.get(i)[1] /*+ currentFlag.position*/;
				flag.timeDurMs = (int) profile.get(i)[0];
			}
		
			flag.profileSlotSelect = 0;
			
			System.out.println("Splice Points " + flag.position + " Time " + flag.timeDurMs);
			
			//Use Velocity?
			flag.velocityOnly = false;
			flag.zeroPos = false;
			
			//Checks
			if (i == 0) {
				flag.zeroPos = false;
				flag.isLastPoint = false;
			}
			if ((i + 1) == totalCount); flag.isLastPoint = true;
			
			m_talon.pushMotionProfileTrajectory(flag);
			//System.out.println("Successful Splice of Profile points");
		}
	}

	public void testProfile() {
		double[] test = {0, 0.625, 2.5, 5.625, 10, 15.625, 22.5, 30.625, 40, 50.625, 62.5, 75.625, 90, 105.625, 122.5, 140.625, 160, 180.625, 202.5, 225.625, 250, 275.625, 302.5, 330.625, 360, 390.625, 442.5, 455.625, 490, 525.625, 562.5, 600.625};
		CANTalon.TrajectoryPoint testFlag = new CANTalon.TrajectoryPoint();
		
		System.out.println("Points" + test.length);
		
		if (m_status.isUnderrun) {
			System.out.println("Motion Profiling Is Underrun");
		}
		for (int j = 0; j < test.length; j++) {
			testFlag.position = test[j];
			testFlag.timeDurMs = 5;
			
			if(m_debugging); testWriteOutput.set(true);
			
			testFlag.profileSlotSelect = 0;
			
			System.out.println("Test Profile " + test[j] + " Test Couunt " + j);
			
			testFlag.velocityOnly = false;
			testFlag.zeroPos = false;
			
			if (j == 0) {
				testFlag.zeroPos = false;
				testFlag.isLastPoint = false;
			}
			
			if ((j + 1) == test.length) {
				testFlag.isLastPoint = true;
				testFlag.velocity = 0;
			}
			
			m_talon.pushMotionProfileTrajectory(testFlag);
			if (m_debugging); testWriteOutput.set(false);
		}
	}
}
