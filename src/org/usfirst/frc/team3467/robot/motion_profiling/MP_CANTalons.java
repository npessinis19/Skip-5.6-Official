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
	
	private String m_name;
	private CANTalon m_talon;
	
	//Debugging IO outputs
	private static DigitalOutput testWriteOutput;
	private static DigitalOutput testExecuteOutput;
	private static DigitalOutput testProcessOutput;
	
	private boolean m_debugging = false;
	public boolean testFlashOn = false;
	private boolean bufferOutOnce = false;

	private MotionProfileStatus m_status;
	
	//Motion Profiling Variables
	private ArrayList <double[]> flags;;
	
	
	public MP_CANTalons(String name, CANTalon talon, boolean debugging) {
		this.m_name = name;
		this.m_talon = talon;
		this.m_debugging = debugging;
		
		flags = new ArrayList <double[]>();
		m_status = new MotionProfileStatus();
		
		if (m_debugging) {
			testWriteOutput = new DigitalOutput(1);
			testExecuteOutput = new DigitalOutput(2);
			testProcessOutput = new DigitalOutput(3);
		}
		
		m_talon.changeControlMode(TalonControlMode.MotionProfile);
		m_talon.getMotionProfileStatus(m_status);
		
		m_talon.processMotionProfileBuffer();
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

	
	//Retrieve Values from the Motion Profile Status Object Instnace
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
		
		if(m_status.btmBufferCnt <= 1) {
			bufferOutOnce = true;
			
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			if (bufferOutOnce && m_status.btmBufferCnt <= 0) {
				finished = true;
			}
			else {
				finished = false;
			}
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
	
	
	
	//Interface Methods
	public void resetMP() { 
		clearMotionProfileTrajectories();
		disableMotionProfiling();
		
		//notifier.stop();
		
		System.out.println("MP Reset");
	}
	
	
	public void startMP(BuildTrajectory trajectory) {
		SmartDashboard.putString("TestProfiling Message", "startMP Called");
		
		resetMP();
		
		System.out.println("Starting Motion Profiling");
		
		startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		
		changeMotionControlFramePeriod(20);
				
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		upDateMotionProfileStatus();
		
		SmartDashboard.putNumber("Init Left Botttom Buffer", BottomBufferCount());
		
		enableMotionProfiling();
	}
	
	
	//Create Motion Profile trajectory points
	public void startFilling(ArrayList <double[]> profile, int totalCount, boolean invert) {

		//Create an empty point
		CANTalon.TrajectoryPoint flag = new TrajectoryPoint();
		
		//Check if in Underrun Condition
		if (m_status.isUnderrun) {
			System.out.println("Motion Profiling Is Underrun");
		}
		
		//Clear isUnderrun Flag
		m_talon.clearMotionProfileHasUnderrun();
		
		/*
		//Hold on initial Point
		initHold.position = 2250;
		initHold.isLastPoint = false;
		m_talon.pushMotionProfileTrajectory(initHold);
		m_talon.processMotionProfileBuffer();
		m_talon.set(2);
		*/
		
		for (int i = 0; i < totalCount; i++) {
				flag.position = profile.get(i)[1];
				flag.timeDurMs = (int) profile.get(i)[0];
		
			//if (m_debugging); testWriteOutput.set(true);
		
		/*
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			*/
			
			//Use PID Slot 1 for Motion Profiling
			flag.profileSlotSelect = 0;
		
			System.out.println("Profile Points " + flag.position + " Time " + flag.timeDurMs + " Index " + i);
			
			//Only use velocities?
			flag.velocityOnly = false;
			flag.zeroPos = false;
		
			if (i == 0) {
				flag.zeroPos = false;
				flag.isLastPoint = false;
			}
			
			if ((i + 1) == totalCount) {
				flag.isLastPoint = true;
			}
			m_talon.pushMotionProfileTrajectory(flag);
			System.out.println("Successful Push of Profile Point " + m_talon.pushMotionProfileTrajectory(flag));
			//testWriteOutput.set(false);
		}
	}

	
	//Splice Motion Profile Trajectory
	public void spliceTrajectory(ArrayList <double[]> profile, int totalCount, boolean invert) {
		
		//Update Status
		m_talon.getMotionProfileStatus(m_status);
		
		//Create an empty point
		CANTalon.TrajectoryPoint spliceFlag = new TrajectoryPoint();
		CANTalon.TrajectoryPoint currentFlag = m_status.activePoint;
		
		//Check if is Underrun
		if (m_status.isUnderrun) {
			System.out.println("Motion Profiling is Underrun");
		}
		
		m_talon.clearMotionProfileHasUnderrun();
		
		for (int i = 0; i < totalCount; i++) {
			spliceFlag.position = profile.get(i)[1] + currentFlag.position;
			spliceFlag.timeDurMs = (int) profile.get(i)[0];
			
			spliceFlag.profileSlotSelect = 0;
			
			System.out.println("Splice Points " + spliceFlag.position + " Time " + spliceFlag.timeDurMs);
			
			spliceFlag.velocityOnly = false;
			spliceFlag.zeroPos = false;
			
			if (i == 0) {
				spliceFlag.zeroPos = false;
				spliceFlag.isLastPoint = false;
			}
			if ((i + 1) == totalCount) {
				spliceFlag.isLastPoint = true;
			}
			
			m_talon.pushMotionProfileTrajectory(spliceFlag);
				System.out.println("Successful Splice of Profile points");
			
			if ((i + 1) <= 128) {
				m_talon.processMotionProfileBuffer();
				System.out.println("Successful");
			}
			
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
