package org.usfirst.frc.team3467.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class GRIP {
	
	public NetworkTable table;

//Vision Processing Constants
	//Is the goal on target
	public boolean imageOnTarget = false;
	public boolean oneContour = false;
	
	boolean onTargetx = false;
	boolean onTargety = false;
	
	//Height of of the top of the Top Target
	private static final int TOP_TARGET_HEIGHT = 97;
	
	//Camera Variables (Axis M1011) Origional Camera
	private static final double M1011_FOVx_deg = 47.0; //Degrees
	private static final double M1011_FOVx_px = 240; //Pixels
	private static final double M1011_Height_ft = 0.979; //Feet
	private static final double M1011_Pitch_deg = 57.0; //Degrees
	
	//Camera Variables (Axis M1013) Second Camera 
	private static final double M1013FOVx = 0.0;
	
	//Target Variables
	private static final double Target_Length_ft = 1.667; //Feet
	private static final double Target_Height_ft = 1.0; //Feet
	
	//Calculated Values
	public double angle_theta;
	public double distance_delta;
	
	final double targetx = 150.1;
	final double targety = 0.0;
	private static final double target_distance = 0.0;
	private static final double target_angle = 0.0;
	
	//Tolerance values
	private static final double TOLERANCE_distance = 5.0;;
	private static final double TOLERANCE_angle = 0.0;
	
	//Image Matricies and Values
	double[] defaultValue = new double[0];
	
	double Centerx;
	double Centery;
	double Height;
	double Width;
	
	public void createImage () {
		
			double[] centerx = table.getNumberArray("centerX", defaultValue);
			double[] centery = table.getNumberArray("centerY", defaultValue);
			double[] width = table.getNumberArray("width", defaultValue);
			double[] height = table.getNumberArray("height", defaultValue);
			
			if (centerx.length == 1) {
				oneContour = true;
				
				Centerx = centerx[0];
				Centery = centery[0];
				Width = width[0];
				Height = height[0];
			}
		}
	
	public double getCenterX() {
		return Centerx;
	}
	
	public double getCenterY() {
		return Centery;
	}
	
	public double getHeight() {
		return Height;
	}
	
	public double getWidth() {
		return Width;
	}
	
	public double calcDistnace( double targetWidth) {
		distance_delta = (Target_Length_ft * M1011_FOVx_px)/(2 * targetWidth * Math.tan(angle_theta));
		SmartDashboard.putNumber("Vision Distance", distance_delta); 
		return distance_delta;
	}
	
	public double calcAngle(double centerX) {
		angle_theta = 47 * Math.cos(120 - (double) centerX);
		SmartDashboard.putNumber("Vision Angle", angle_theta);
		return angle_theta;
	}
	
	public boolean isOnTarget( double distance, double angle) {
		double deltaDistance = Math.abs(distance - target_distance);
		double deltaAngle = Math.abs(angle - target_angle);
		if ((deltaDistance >= 0 && deltaDistance <= TOLERANCE_distance) &&
				deltaAngle >= 0 && deltaAngle <= TOLERANCE_angle) {
			imageOnTarget = true;
		}
		SmartDashboard.putBoolean("Image On Target", imageOnTarget);
		return imageOnTarget;
	}
}	
