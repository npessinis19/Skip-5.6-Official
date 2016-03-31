package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.pid.PIDF_CANTalon;
import org.usfirst.frc.team3467.robot.subsystems.Vision.GRIP;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetGoal extends CommandBase {

	private PIDController vision_PID;
	private PIDF_CANTalon leftpidf_drive;
	private PIDF_CANTalon rightpidf_drive;
	
	//Position PID Constants
	private static final double Posit_P = 1.0;
	private static final double Posit_I = 0.0;
	private static final double Posit_D = 0.0;
	private static final double Posit_TOLERANCE = 1.0;

	//Angle/Vision PID Constants
	private static double Angle_P = 1.0;
	private static double Angle_I = 0.0;
	private static double Angle_D = 0.0;
	private static final double Angle_TOLERANCE = 1.0;
	
	//Management Variables
	private double position; //SetPoints for CANTalons
	private boolean onTarget; //Is the robot on target
	private boolean stop = false; //Turns off command when in tolerance zone
	private double oldAngle_P = Angle_P; //Keep track of old Angle_P values
	private int count = 0; //Count when robot is in tolerance zone
	private static final int sufficientCount = 28; //Count limit, when reached, turn off command
	
	private static final boolean debugging = false;
	
	public TargetGoal() {
	//Build_Controllers();
	//setTimeout(4);
	}
	
	public void Build_Controllers() {
		vision_PID = new PIDController(Angle_P, Angle_I, Angle_D, 
				new PIDSource() {
					PIDSourceType vision_SourceType = PIDSourceType.kDisplacement;
				
				public double pidGet() {
					return ahrs.getGyroAngle();
				}
				
				public void setPIDSourceType(PIDSourceType VisionTargetingPID) {
					vision_SourceType = VisionTargetingPID;
				}
				
				public PIDSourceType getPIDSourceType(){
					return vision_SourceType;
				}
				
		},
		new PIDOutput() {
				public void pidWrite(double output) {
					double error = vision_PID.getError();
					position = output * 14.75;
					
					if (Math.abs(error) >= 0 && Math.abs(error) <= Angle_TOLERANCE) {
						if (Angle_P > 0.01) {
							Angle_P = oldAngle_P - 0.3 * (count + 1);
							if (Angle_P < 0.01) Angle_P = 0.01;
							
							vision_PID.setPID(Angle_P, Angle_I, Angle_D);
							oldAngle_P = Angle_P;
						}
						
						System.out.println("Angle_P " + Angle_P);
						System.out.println("Count " + count);
						
						if (count++ < sufficientCount) {
							stop = true;
						}
					}
					else {
					//Right Encoder is backwards
					leftpidf_drive.setSetpoint(position);
					//rightpidf_drive.setSetpoint(position / 2);
					}
				}});
		vision_PID.setAbsoluteTolerance(Angle_TOLERANCE);
		
		leftpidf_drive = new PIDF_CANTalon("Left Drive", driveBase.getLeftTalon(), Posit_TOLERANCE, false, false);
		leftpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
		
		rightpidf_drive = new PIDF_CANTalon("Right Drive",  driveBase.getRightTalon(), Posit_TOLERANCE, false, false );
		rightpidf_drive.setPID(Posit_P, Posit_I, Posit_D);
	}
	
	public void Set_Angle() {
		vision_PID.setSetpoint(grip.getChangeinAngle());
	}
	
	public void Start_PID() {
		vision_PID.enable();
		
		leftpidf_drive.enable();
		rightpidf_drive.enable();
	}
	
	public void Setup_PID() {
		vision_PID.setSetpoint(ahrs.getGyroAngle());
		leftpidf_drive.setSetpoint(driveBase.getDistance());
		rightpidf_drive.setSetpoint(driveBase.getDistance());
	}
	
	public void reset_PID() {
		vision_PID.reset();
		
		leftpidf_drive.reset();
		rightpidf_drive.reset();
	}
	
	public void End_PID() {
		vision_PID.disable();
		
		leftpidf_drive.disable();
		rightpidf_drive.disable();
	}
	
	public void deBug() {
		if (debugging) {
			double A_p = SmartDashboard.getNumber("Vision: P");
			double A_i = SmartDashboard.getNumber("Vision: I");
			double A_d = SmartDashboard.getNumber("Vision: D");
			
			vision_PID.setPID(Angle_P, A_i, A_d);
		}
	}
	
	protected void initialize() {
		ahrs.gyroReset();
		driveBase.resetEncoders();
		
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
		
		//reset_PID();	
	}

	protected void execute() {
		/*if (!grip.isGoodImage()) {
			grip.createImage();
		}
		else {
			grip.calculateTargetData();
			
			//for (int j = 0; j < 1; j++) {
			//	Set_Angle();
			//}
		}
		*/
		
		grip.createImage();
		grip.calculateTargetData();
		grip.printData();
		ahrs.reportGyroValues();
		driveBase.reportEncoders();
		
		deBug();
		
		SmartDashboard.putNumber("Vision: Change in Angle", grip.getChangeinAngle());
		SmartDashboard.putNumber("Vision: Change in Distance", grip.getChangeinDistance());
	}

	protected boolean isFinished() {
		//return isTimedOut() || grip.isOnTarget();
		return isTimedOut();
	}

	protected void end() {
		//End_PID();
		//reset_PID();
	}

	protected void interrupted() {
		end();
	}

}
