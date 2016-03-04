package org.usfirst.frc.team3467.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class GRIP {
	
	public NetworkTable table;

//Vision Processing Constants
	//Is the goal on target
	public boolean imageOnTarget = false;
	
	boolean onTargetx = false;
	boolean onTargety = false;
	
	//Height of of the top of the Top Target
	public static final int TOP_TARGET_HEIGHT = 97;
	
	//Camera angles are different for different camera
	public static final double M1011FOVx = 47.0;
	public static final double M1011Height = 11.75; //Inches
	public static final double M1011Pitch = 57.0; //Degrees
	
	
	
	public static final double M1013FOVx = 0.0;
	
	final double targetx = 150.1;
	final double targety = 0.0;
	
	private final double TOLERANCEx = 0.0;
	private final double TOLERANCEy = 0.0;
	
	//Image Matricies
	double[] functionx = new double [0];
	double[] functiony = new double [0];
	double[] defaultValue = new double[0];
	
	public void createImage () {
			
			double[] centerx = table.getNumberArray("centerX", defaultValue);
			double[] centery = table.getNumberArray("centerY", defaultValue);
		
		if  (centerx.length == 0) {
				functionx[0] = centerx[0];
				functiony[0] = centery[0];
				
				if(functionx[0] >= targetx - TOLERANCEx && functionx[0] <= targetx + TOLERANCEx) {
					onTargetx = true;
				}
				
				if (functiony[0] >= targety - TOLERANCEy && functiony[0] <= targety + TOLERANCEy) {
					onTargety = true;
				}
			}
			
			if (onTargetx && onTargety) {
				imageOnTarget = true;
			}
		}
	
	public double getCenterX() {
		return functionx[0];
	}
	
	public double getCenterY() {
		return functiony[0];
	}
	
	public double calcDistnace() {
		return 0.0;
	}
	
	public double calcAngle() {
		double angle;
		return 0.0;
	}
	
	public boolean onGoalx() {
		return onTargetx;
	}
	
	public boolean onGoaly() {
		return onTargety;
	}
}	
