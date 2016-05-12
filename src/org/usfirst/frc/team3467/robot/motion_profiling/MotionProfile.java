package org.usfirst.frc.team3467.robot.motion_profiling;

import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;

public interface MotionProfile {

	public void resetMP();
	
	public void startMP(BuildTrajectory trajectory);
	
	public void publishValues();
}
