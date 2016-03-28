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
	private static final double M1011_FOVx_px = 320; //Pixels
	private static final double M1011_Height_ft = 0.979; //Feet
	private static final double M1011_Pitch_deg = 57.0; //Degrees
	
	//Camera Variables (Axis M1013) Second Camera 
	private static final double M1013_FOVx_deg = 0.0;
	
	//Target Variables
	private static final double Target_Length_ft = 1.667; //Feet
	private static final double Target_Height_ft = 1.0; //Feet
	
	//Calculated Values
	private double angle_theta = 0.0;
	private double distance_delta = 0.0;
	private double changeinDistance = 0.0;
	private double changeinAngle = 0.0;
	
	private static final double targetx = 150.1;
	private static final double targety = 0.0;
	private static double target_distance = 13.0;
	private static double target_angle = 0.0;
	
	//Tolerance values
	private static final double TOLERANCE_distance = 5.0;;
	private static final double TOLERANCE_angle = 0.5;
	
	//Image Matricies and Values
	private double[] defaultValue = new double[0];
	
	private double Centerx = 0.0;;
	private double Centery = 0.0;
	private double Height = 0.0;
	private double Width = 0.0;
	private int contours = 0;
	
	public GRIP() {
		table = NetworkTable.getTable("GRIP/myContoursReport");
	}
	
	//Get values from Network Table and work with those values
	public void createImage () {
			double[] centerx = table.getNumberArray("centerX", defaultValue);
			double[] centery = table.getNumberArray("centerY", defaultValue);
			double[] width = table.getNumberArray("width", defaultValue);
			double[] height = table.getNumberArray("height", defaultValue);
			
			contours = centerx.length;
		
				Centerx = centerx[0];
				Centery = centery[0];
				Width = width[0];
				Height = height[0];
				
			//Print values to SmartDashboard
				SmartDashboard.putNumber("Vision: Centerx", Centerx);
				SmartDashboard.putNumber("Vision: Centery", Centery);
				SmartDashboard.putNumber("Vision: Width", Width);			
				SmartDashboard.putNumber("Vision: Contours", centerx.length);
	}
	
	public boolean isGoodImage() {
		if(contours == 1) {
			return true;
		}
		else {
			return false;
		}
	}
	
	//Get the specific values from the network table
	public double getCenterX() {
		return this.Centerx;
	}
	
	public double getCenterY() {
		return this.Centery;
	}
	
	public double getHeight() {
		return this.Height;
	}
	
	public double getWidth() {
		return this.Width;
	}
	
	public double getAngle_theta() {
		return angle_theta;
	}
	
	public double getDistance_delta() {
		return distance_delta;
	}
	
	public double getChangeinAngle() {
		return changeinAngle;
	}
	
	public double getChangeinDistance() {
		return changeinDistance;
	}
	
	
	//Calibrate where the target should be
	public void setTarget_distnce(double distance) {
		target_distance = distance;
	}
	
	public void setTarget_angle(double angle) {
		target_angle = angle;
	}
	

	public void printData() {
		SmartDashboard.putNumber("Vision: Distance", distance_delta);
		SmartDashboard.putNumber("Vision: Angle", angle_theta);
		SmartDashboard.putNumber("Vision: Change in Angle", changeinAngle);
		SmartDashboard.putNumber("Vision: Change in Distance", changeinDistance);
	}
	
	
	//Calculations of distance and of position
	public boolean isOnPixel() {
		boolean onPixel = false;
		if ((Centerx - targetx) >=0 && (Centerx - targetx) <= 1/*Pixel Tolerance*/) {
			if (Centery - targety >= 0 && Centery - targety <=1/*Pixel Tolerance*/) {
				onPixel = true;
			}
		}
		return onPixel;
	}
	
	public void calculateTargetData() {
		imageOnTarget = false;
		
		//Calculates The Distance From the Target
		distance_delta = (Target_Length_ft * M1011_FOVx_px)/(2 * Width * Math.tan(angle_theta));
		
		//Calculates The Angle of the Target
		angle_theta = ((Centerx - M1011_FOVx_px/2)/(M1011_FOVx_px/2)) * M1011_FOVx_deg;
		
		//Calculate the distances and angles needed to move
		changeinDistance = distance_delta - target_distance;
		changeinAngle = (target_angle - angle_theta) - 10;
		
		//Prints the distances and angles needed to move to SmartDashBoard
		SmartDashboard.putNumber("Vision: Distance", distance_delta);
		SmartDashboard.putNumber("Vision: Angle", angle_theta);

		SmartDashboard.putNumber("Vision: Change in Angle", changeinAngle);
		SmartDashboard.putNumber("Vision: Change in Distance", changeinDistance);
	}
	
	public boolean isOnTarget() {
		if ((Math.abs(changeinDistance) >= 0 && Math.abs(changeinDistance) <= TOLERANCE_distance) &&
			 Math.abs(changeinAngle) >= 0 && Math.abs(changeinAngle) <= TOLERANCE_angle) {
			imageOnTarget = true;
		}
		else {
			imageOnTarget = false;
		}
		
		SmartDashboard.putBoolean("Image On Target", imageOnTarget);
		
		return imageOnTarget;
	}
}	
