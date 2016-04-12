package org.usfirst.frc.team3467.robot.motion_profiling;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BuildTrajectory {

	private static ArrayList<double[]> flags;
	
	
	public BuildTrajectory(int xnet, double accel, double decel, double cruise, double step) {
		flags = TrapProfGen(xnet, accel, decel, cruise, step);
	}
	
	
	//Get Profile Values
	public ArrayList <double[]> getprofile() {
		return flags;
	}
	
	public double getTime(int i) {
		return flags.get(i)[0];
	}
	
	public double getDistance(int i) {
		return flags.get(i)[1];
	}
	
	public int getTotalCount() {
		return flags.size();
	}
	

	/* 
	 * Creates the Profile based on input values
	 * (xnet) is the total distance in encoder ticks
	 * (accel) is the total acceleration in ticks/ms^2
	 * (decel) is the total decceleration in ticks/ms^2
	 * (cruise) is the cruise velocity in ticks/ms
	 * (step) is the period between profile points in ms
	 */
	public ArrayList<double[]> TrapProfGen(int xnet, double accel, double decel, double cruise, double step) {
		
		SmartDashboard.putString("BuildTrajectory Message", "Profile Generation Started");
		
		ArrayList <double[]> out = new ArrayList <double[]>();
		
		boolean toCruise = true;
		double time =  0;
		double taccel = cruise/accel;
		double tdecel = cruise/decel;
		double xaccel = Math.pow(cruise, 2)/(2 * accel);
		double xdecel = Math.pow(cruise, 2)/(2 * decel);
		double xcruise = xnet - xaccel - xdecel;
		double tcruise = xcruise/cruise;
		double vaccel = 0;
		
		if (xaccel + xdecel >= xnet) {
			tdecel = Math.sqrt(xnet/(.5*decel+Math.pow(decel, 2)/(2*accel)));
			taccel = tdecel*decel/accel;
			xaccel = .5 * accel * Math.pow(taccel, 2);
			tcruise = 0;
			xcruise = 0;
			cruise = 0;
			vaccel = accel * taccel;
			toCruise = false;
		}
		
		while (time <= tcruise + taccel + tdecel) {
			double x;
		
			if (time < taccel) {
				x = .5 * accel * Math.pow(time, 2);
			}
			
			else if (time >= taccel && time < tcruise + taccel &&toCruise == true) {
				x = xaccel + cruise*(time-taccel);
			}
			
			else if (toCruise) {
				x = cruise*(time-taccel-tcruise) - .5 * decel * Math.pow(time-taccel-tcruise, 2) + xaccel + xcruise;
			}
			
			else {
				x = vaccel*(time-taccel) - .5 * decel * Math.pow(time-taccel, 2) + xaccel;
			}
		
		double[] outArray = {step , x};
		
		//System.out.println(time+"    "+
		out.add(outArray);
		time = time + step;
		}
		
		double[] outArrayFinal = {step , xnet};
		System.out.println(time+"    "+ xnet);
		out.add(outArrayFinal);
		return out;
		}
}
