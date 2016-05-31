package org.usfirst.frc.team3467.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3467.robot.RobotMap;
import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands.ArcadeDrive;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.motion_profiling.MotionProfile;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout.PowerLevel;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.PowerConsumer;

public class DriveBase extends Subsystem implements PowerConsumer, MotionProfile {
	
	//Use to Toggle Arcade Drive, and t_useTank Drive
	private boolean t_useTank = true;
	
	//CANTalons objects and RobotDrive object
	private CANTalon 		leftTalon, rightTalon, leftTalon2, rightTalon2, leftTalon3, rightTalon3;
	private MP_CANTalons	leftmp_drive, rightmp_drive;
	private static RobotDrive 		t_drive;
	private CANTalon.ControlMode 	t_controlMode;

	//Instance of the DriveBase Class
	private static DriveBase 		instance;
	
	
	//DriveBase get instances
	public DriveBase getInstance() {
		return instance;
	}

	public CANTalon getLeftTalon() {
		return leftTalon;
	}
	
	public CANTalon getRightTalon() {
		return rightTalon;
	}
	
	
	//Initializing the Default Command
	public void initDefaultCommand() {
		//this.setDefaultCommand(new ArcadeDrive());
		System.out.println("DriveBase: Set to ArcadeDrive");
	}
	
	
	//DriveBase class constructor
	public DriveBase() {
		instance = this;
		
		leftTalon = new CANTalon(RobotMap.drivebase_LeftTalon);
		rightTalon = new CANTalon(RobotMap.drivebase_RightTalon);
		leftTalon2 = new CANTalon(RobotMap.drivebase_LeftTalon2);
		rightTalon2 = new CANTalon(RobotMap.drivebase_RightTalon2);
		leftTalon3 = new CANTalon(RobotMap.drivebase_LeftTalon3);
		rightTalon3 = new CANTalon(RobotMap.drivebase_RightTalon3);
		
		//Instantiate MP_CANTalon
		leftmp_drive = new MP_CANTalons("Left Drive", leftTalon, false);
		rightmp_drive = new MP_CANTalons("Right Drive", rightTalon, false);
		
		//Slave extra Talons on each side
		setSlaveMode(true);
		
		// Turn off Brake mode
		setTalonBrakes(false);
		
		//Set default control Modes for Master CANTalons
		setControlMode(TalonControlMode.PercentVbus);
		
		//Set SIM encoders as feedback devices
		leftTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rightTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		
		//Instantiate RobotDrive
		t_drive = new RobotDrive(leftTalon, rightTalon);
		
		//RobotDrive Parameters
		t_drive.setSafetyEnabled(true);
		t_drive.setExpiration(1.0);
		t_drive.setSensitivity(0.5);
		t_drive.setMaxOutput(1.0);
	}
	
	
	//Called for a PowerLevel update (See Brownout)
	public void callbackAlert(Brownout.PowerLevel level) {
		switch (level) {
		case Normal:
				leftTalon.configMaxOutputVoltage(12.0);
				rightTalon.configMaxOutputVoltage(12.0);
			break;
		case Chill:
				leftTalon.configMaxOutputVoltage(9.0);
				rightTalon.configMaxOutputVoltage(9.0);
			break;
		case Critical:
				leftTalon.configMaxOutputVoltage(6.0);
				rightTalon.configMaxOutputVoltage(6.0);
			break;
		}
	}
	
	
	// Set drive mode
	public void setDriveMode(boolean usetank) {
		t_useTank = usetank;
	}
	
	//Set up for normal Drive mode
	public void initDrive () {
		if (t_controlMode != TalonControlMode.PercentVbus); {
				leftTalon.changeControlMode(TalonControlMode.PercentVbus);
				rightTalon.changeControlMode(TalonControlMode.PercentVbus);
				


				t_controlMode = TalonControlMode.PercentVbus;
		}			
				// Extra CIMs are slaves
				setSlaveMode(true);
				
				// Brakes are off
				setTalonBrakes(false);
		// Don't need to invert because the sticks give negative values
		// in the forward direction
		leftTalon.setInverted(false);
		rightTalon.setInverted(false);
	}
	
	//Use Standard Tank Drive method
	public void driveTank (double LeftTalon, double RightTalon, boolean squared) {
		t_drive.tankDrive(LeftTalon, RightTalon, squared);
		if (true) {
			reportEncoders();
		}
	}

	// Use single-stick Arcade Drive method
	public void driveArcade(double move, double rotate, boolean square) {
		t_drive.arcadeDrive(move, rotate, square);
		if (true) {
			reportEncoders();
			CommandBase.ahrs.reportGyroValues();
		}
	}

	// pass-thru to RobotDrive drive() method (used in autonomous)
	public void drive(double outputMagnitude, double curve) {
		t_drive.drive(outputMagnitude, curve);
	}

	/**
	 * Toggles the following and PercentVbus control modes of ALL BUT THE
	 * left and right master CANTalons
	 * 
	 * @param enslave Enslave the CANTalons?
	 */
	public void setSlaveMode(boolean enslave) {
		if (enslave == false) {
			leftTalon2.changeControlMode(TalonControlMode.PercentVbus);
			leftTalon3.changeControlMode(TalonControlMode.PercentVbus);
			rightTalon2.changeControlMode(TalonControlMode.PercentVbus);
			rightTalon3.changeControlMode(TalonControlMode.PercentVbus);

			leftTalon2.set(0.0);
			leftTalon3.set(0.0);
			rightTalon2.set(0.0);
			rightTalon3.set(0.0);
		}
		else {
			leftTalon2.changeControlMode(TalonControlMode.Follower);
			leftTalon3.changeControlMode(TalonControlMode.Follower);
			rightTalon2.changeControlMode(TalonControlMode.Follower);
			rightTalon3.changeControlMode(TalonControlMode.Follower);
		
			leftTalon2.set(RobotMap.drivebase_LeftTalon);
			leftTalon3.set(RobotMap.drivebase_LeftTalon);
			rightTalon2.set(RobotMap.drivebase_RightTalon);
			rightTalon3.set(RobotMap.drivebase_RightTalon);
		}
	}
	
	/**
	 * @param controlMode Set the control mode of the left and
	 * right master CANTalons
	 */
	public void setControlMode(TalonControlMode controlMode) {
		leftTalon.changeControlMode(controlMode);
		rightTalon.changeControlMode(controlMode);
		
		// Save control mode so we will know if we have to set it back later
		t_controlMode = controlMode;
	}
	
	/**
	 * @return The current CANTalon control mode
	 */
	public String getTalonControlMode() {
		if (t_controlMode == TalonControlMode.PercentVbus) {
			return "PercentVbus";
		}
		else if (t_controlMode == TalonControlMode.MotionProfile) {
			return "MotionProfile";
		}
		else if(t_controlMode == TalonControlMode.Position) {
			return "Position";
		}
		else; return "Problem";
	}
	
	/**
	 * @return Average of the encoder values from the left and right encoders
	 */
	public double getDistance() {
		return ((leftTalon.getPosition()) + (rightTalon.getPosition() * -1.0))/2;
	}

	public void reportEncoders() {
		SmartDashboard.putNumber("Left Encoder", leftTalon.getPosition());
		SmartDashboard.putNumber("Right Encoder", rightTalon.getPosition() * -1.0);			
	}

	public void resetEncoders() {
		leftTalon.setPosition(0);
		rightTalon.setPosition(0);
	}
	
	/**
	 * Sets the brake mode for ALL CANTalons
	 * @param setBrake Enable brake mode?
	 */
	public void setTalonBrakes(boolean setBrake) {
		leftTalon.enableBrakeMode(setBrake);
		rightTalon.enableBrakeMode(setBrake);
		leftTalon2.enableBrakeMode(setBrake);
		rightTalon2.enableBrakeMode(setBrake);
		leftTalon3.enableBrakeMode(setBrake);
		rightTalon3.enableBrakeMode(setBrake);
	}

	
	//Motion Profile Interface Buffer
	/**
	 * Clears motion profile trajectories from the top and bottom buffers,
	 * and stop the motion profile executer
	 */
	public void resetMP() {
		leftmp_drive.clearMotionProfileTrajectories();
		rightmp_drive.clearMotionProfileTrajectories();
		
		leftmp_drive.disableMotionProfiling();
		rightmp_drive.disableMotionProfiling();
	}

	/**
	 * Creates motion profile trajectory from a BuildTrajectory object, and setting the rate of processing
	 * (moving trajectory points from top to bottom buffer)
	 */
	public void startMP(BuildTrajectory trajectory) {
		leftmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		rightmp_drive.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		
		leftmp_drive.changeMotionControlFramePeriod(20);
		rightmp_drive.changeMotionControlFramePeriod(20);
	}
	
	/**
	 * Starts the motion profile executer
	 */
	public void enableMP() {
		leftmp_drive.enableMotionProfiling();
		rightmp_drive.enableMotionProfiling();
	}

	public void publishValues() {
		SmartDashboard.putNumber("Left Top Buffer", leftmp_drive.topBuffercount());
		SmartDashboard.putNumber("Right Top Buffer", rightmp_drive.topBuffercount());
		
		SmartDashboard.putNumber("Left Bottom Buffer", leftmp_drive.BottomBufferCount());
		SmartDashboard.putNumber("Right Bottom Buffer", rightmp_drive.BottomBufferCount());
		
		SmartDashboard.putBoolean("Left Is Underrun", leftmp_drive.isUnderrun());
		SmartDashboard.putBoolean("Right Is Underrun", rightmp_drive.isUnderrun());
		
		SmartDashboard.putBoolean("Left Has Underrun", leftmp_drive.hasUnderrun());
		SmartDashboard.putBoolean("Right Has Underrun", rightmp_drive.hasUnderrun());
	
		System.out.println("Left Active Point" + leftmp_drive.getActivePoint().position);
		//"Right Active Point", rightmp_drive.getActivePoint().position);
	}

	/**
	 * Move 1 motion profile trajectory point from the top API buffer
	 * to the bottom CANTalon buffer
	 */
	public void processMotionProfileBuffer() {
		leftmp_drive.processMotionProfileBuffer();
		rightmp_drive.processMotionProfileBuffer();
	}

	/**
	 * Update motion profile status object to retrieve motion profile data from
	 * the CANTalon
	 */
	public void updateMotionProfileStatus() {
		leftmp_drive.upDateMotionProfileStatus();
		rightmp_drive.upDateMotionProfileStatus();
	}

	/**
	 * Checks if the motion profile is complete
	 */
	public boolean isComplete() {
		if (leftmp_drive.isComplete() && rightmp_drive.isComplete()) {
			return true;
		}
		else {
			return false;
		}
	}
}
