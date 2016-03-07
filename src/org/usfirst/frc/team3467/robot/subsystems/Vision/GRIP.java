package org.usfirst.frc.team3467.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class GRIP {
	
	public NetworkTable table;

//Vision Processing Constants
	//Is the goal on target
	public boolean imageOnTarget = false;

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
	public double angle_theta = 0.0;
	public double distance_delta = 0.0;
	public double changeinDistance = 0.0;
	public double changeinAngle = 0.0;
	
	private static final double targetx = 150.1;
	private static final double targety = 0.0;
	private static final double target_distance = 0.0;
	private static final double target_angle = 0.0;
	
	//Tolerance values
	private static final double TOLERANCE_distance = 5.0;;
	private static final double TOLERANCE_angle = 0.0;
	
	//Image Matricies and Values
	private double[] defaultValue = new double[0];
	
	private double Centerx;
	private double Centery;
	private double Height;
	private double Width;
	
	public void createImage () {
		
			double[] centerx = table.getNumberArray("centerX", defaultValue);
			double[] centery = table.getNumberArray("centerY", defaultValue);
			double[] width = table.getNumberArray("width", defaultValue);
			double[] height = table.getNumberArray("height", defaultValue);
			
			if (centerx.length == 1) {
				
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
	
	public boolean isOnPixel() {
		boolean onPixel = false;
		if ((Centerx - targetx) >=0 && (Centerx - targetx) <= 1/*Pixel Tolerance*/) {
			if (Centery - targety >= 0 && Centery - targety <=1/*Pixel Tolerance*/) {
				onPixel = true;
			}
		}
		return onPixel;
	}
	
	public boolean isOnTarget() {
		//Calculates The Distance From the Target
		distance_delta = (Target_Length_ft * M1011_FOVx_px)/(2 * Width * Math.tan(angle_theta));
		
		//Calculates The Angle of the Target
		angle_theta = 47 * Math.cos(120 - Centerx);
		
		//Calculate the distances and angles needed to move
		changeinDistance = distance_delta - target_distance;
		 changeinAngle = target_angle - angle_theta;
		
		//Prints Values to SmartDashBoard
		SmartDashboard.putNumber("Vision: Distance", distance_delta);
		SmartDashboard.putNumber("Vision: Angle", angle_theta);
		
		 
		if ((Math.abs(changeinDistance) >= 0 && Math.abs(changeinDistance) <= TOLERANCE_distance) &&
				Math.abs(changeinAngle) >= 0 && Math.abs(changeinAngle) <= TOLERANCE_angle) {
			imageOnTarget = true;
		}
		SmartDashboard.putNumber("Image: Change in Angle", changeinAngle);
		SmartDashboard.putNumber("Image: Change in Distance", changeinDistance);
		SmartDashboard.putBoolean("Image On Target", imageOnTarget);
		return imageOnTarget;
	}
}	
