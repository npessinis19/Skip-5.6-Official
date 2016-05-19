package org.usfirst.frc.team3467.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3467.robot.RobotMap;
import org.usfirst.frc.team3467.robot.commands.CommandBase;
import org.usfirst.frc.team3467.robot.motion_profiling.BuildTrajectory;
import org.usfirst.frc.team3467.robot.motion_profiling.MP_CANTalons;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.PowerConsumer;
import org.usfirst.frc.team3467.robot.subsystems.Shooter.commands.ShooterReset;
import org.usfirst.frc.team3467.robot.motion_profiling.MotionProfile;

/*
	How the Shooter works:
	
	Manual mode:  Left stick Y axis controls the reset bar.
	
	PID mode: PID loop runs continuously and responds to changes in setpoint 
	made by calls to latch() and clear()
 
 */

public class Shooter extends PIDSubsystem implements PowerConsumer, MotionProfile {
	
	private MP_CANTalons shooterTalon;
	
	// Controls display to SmartDashboard
	private static final boolean debugging = true;
	
	// Talon PDP channel number
	private static final int WINCH_PDP_CHANNEL = 0;
	
	//Catapult Objects
	private CANTalon m_resetBar;
	private DoubleSolenoid m_catLatch;
	//private AnalogPotentiometer m_resetAngle;
	
	//Motion Profile Trajectories
	public BuildTrajectory shooterTrajectory;
	
	//PID Constants
	private static final double SHOOT_P = 20.0;
	private static final double SHOOT_I = 0.5;
	private static final double SHOOT_D = 0.0;
	
	private static final double TOLERANCE = 0.01;
	
	// PID variables
	private int m_resetBarSetpoint;
	private boolean m_usePID;

	// Maximum allowed current for resetting the catapult
	private static final double MAX_RESET_CURRENT = 50.0;

	// Power flags
	private boolean m_voltageTooLow = false;
	private boolean m_currentTooHigh = false;
	private boolean m_powerAlert = false;
	
	// Reset bar setpoints
	private int m_clearPoint = 490;  // bar is out of the way of catapult
	private int m_latchPoint = 61; // bar is holding catapult so it can be latched
	private int m_deltaPot = 429; //Change in pot signal between clear and latch points
	// The roboRio Preferences
	Preferences m_prefs = Preferences.getInstance();
	
	// Has the robot been calibrated
	private static boolean m_hasBeenCalibrated = false;
	
	
	//Shooter Constructor
	public Shooter() {
		
		super("Shooter", SHOOT_P, SHOOT_I, SHOOT_D);

		//m_resetAngle = new AnalogPotentiometer(new AnalogInput(RobotMap.catapult_potentiometer_port));
		m_resetBar = new CANTalon(RobotMap.catapult_Talon);
		
		shooterTalon = new MP_CANTalons("Shooter", m_resetBar, false);
		
		m_resetBar.enableBrakeMode(true);
		m_resetBar.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		
		m_catLatch = new DoubleSolenoid(RobotMap.catapult_solenoid_latch, RobotMap.catapult_solenoid_release);
		
		// Start with setpoint at the current potentiometer reading 
		m_resetBarSetpoint = m_resetBar.getAnalogInRaw();
		m_usePID = false;
		
		this.setAbsoluteTolerance(TOLERANCE);
		
		// Update reset bar setpoints from Preferences
		int cp = m_clearPoint; int lp = m_latchPoint;
		m_clearPoint = m_prefs.getInt("Shooter Clear Point", cp);
		m_latchPoint = m_prefs.getInt("Shooter Latch Point", lp);
		
		// Update PID gains from Preferences
		double p, i, d;
		p = m_prefs.getDouble("Shooter P Gain", SHOOT_P);
		i = m_prefs.getDouble("Shooter P Gain", SHOOT_I);
		d = m_prefs.getDouble("Shooter P Gain", SHOOT_D);
		this.getPIDController().setPID(p, i, d);		
		
		// Register with Brownout subsystem
		Brownout.getInstance().registerCallback(this);	
		
		//Instantiate trajectories
		shooterTrajectory = new BuildTrajectory(429, 3, 3, 5, 10);
	}

	protected void initDefaultCommand() {
		this.setDefaultCommand(new ShooterReset());	// Drive Manually by default
	}
	
	public CANTalon returnCANTalon(){
		return m_resetBar;
	}
	
	//Set Shooter Modes
	public void initManualMode() {
		
		if (m_usePID) {
			m_usePID = false;
			this.disable();
			
			// Stop motor until we are ready to set speed
			m_resetBar.set(0);
			
			if (debugging)
		    	SmartDashboard.putBoolean("Shooter PID Enabled", false);
		}
	}
	
	public void driveManual(double speed) {
		
		// Drive reset bar at commanded speed,
		// being sure to check for limit switches
		m_resetBar.set(check4Endpoints(speed));	

		// Update the reset bar setpoint even while in manual mode
		// to avoid surprises when returning to PID control
		int angle = m_resetBar.getAnalogInRaw();
		if (debugging) {
	    	SmartDashboard.putNumber("Shooter Reset Angle", angle);
		}
		m_resetBarSetpoint = angle;
	}

	public boolean initPIDMode() 
	{
		if (!m_usePID) {
			m_usePID = true;

			this.setSetpoint(m_resetBarSetpoint);
			this.enable();
		
			if (debugging)
		    	SmartDashboard.putBoolean("Shooter PID Enabled", true);
		}
		return true;
	}
	
	
	/*
	 * Methods to move Reset Bar to useful positions
	 */
	public void latch() {

		this.setSetpoint(m_latchPoint);
		
		// Save the position
		m_resetBarSetpoint = m_latchPoint;
		
		if (debugging) {
			SmartDashboard.putNumber("Shooter Setpoint", m_resetBarSetpoint);
			SmartDashboard.putBoolean("Shooter Current Too High", m_currentTooHigh);
			SmartDashboard.putBoolean("Shooter Voltage Too Low", m_voltageTooLow);
		}
	}
	
	public void clear() {

		this.setSetpoint(m_clearPoint);
		
		// Save the position
		m_resetBarSetpoint = m_clearPoint;
		
		if (debugging)
			SmartDashboard.putNumber("Shooter Setpoint", m_resetBarSetpoint);
	}
	
	
	//Check if if the reset bar is in a position
	public boolean resetBarIsClear() {
		// If reset angle is >= m_clearPoint, or if clear limit switch is closed, return true
		return ((m_resetBar.getAnalogInRaw() >= m_clearPoint) || m_resetBar.isFwdLimitSwitchClosed());
	}
	
	public boolean resetBarIsLatched() {
		return (m_resetBar.getAnalogInRaw() <= m_latchPoint  || m_resetBar.isRevLimitSwitchClosed());
	}
	
	public int getResetAngle() {
		return m_resetBar.getAnalogInRaw();
	}
	
	
	
	// Control the solenoid that latches the catapult
	public void cataLatch() {
		m_catLatch.set(DoubleSolenoid.Value.kForward);
	}
	
	public void cataShoot() {
		m_catLatch.set(DoubleSolenoid.Value.kReverse);		
	}

	public void cataStop() {
		m_catLatch.set(DoubleSolenoid.Value.kOff);		
	}

	// PowerConsumer callback
	public void callbackAlert(Brownout.PowerLevel level) {
		switch (level) {
		case Critical:
			m_voltageTooLow = true;
			break;
		default:
			m_voltageTooLow = false;
			break;
		}
	}
	
	// Current Check
	public void checkCurrent() {

		if (CommandBase.brownout.getCurrent(0) > MAX_RESET_CURRENT) {
			m_currentTooHigh = true;
		} else {
			m_currentTooHigh = false;
		}
		
	}
	
	public boolean checkBrownOut() {
		
		boolean retVal = false;
		
		if (m_voltageTooLow || m_currentTooHigh) {
			m_powerAlert = true;
			retVal = true;
		} else {
			retVal = false;			
		}

		// Use m_powerAlert as a flag to know when to update the SmartDashboard
		// (We don't want to spend time updating it if nothing is changing)
		if (debugging && m_powerAlert) {
			SmartDashboard.putBoolean("Shooter Current Too High", m_currentTooHigh);
			SmartDashboard.putBoolean("Shooter Voltage Too Low", m_voltageTooLow);
		}
		m_powerAlert = retVal;

		return retVal;
	}
	
	// This is called during RobotInit
	public void cataCalibrate() {
		
		m_clearPoint = m_resetBar.getAnalogInRaw();
		m_latchPoint = m_clearPoint - m_deltaPot;
		
		m_prefs.putDouble("Shooter Clear Point", m_clearPoint);
		m_prefs.putDouble("Shooter Latch Point", m_latchPoint);
		
		m_hasBeenCalibrated = true;
	}
	
	
	// Has the robot been calibrated before?
	public boolean hasBeenCalibrated() {
		return m_hasBeenCalibrated;
	}
	
	// Check Clear catapult limit switch
	public boolean checkClearLimit() {
		return m_resetBar.isFwdLimitSwitchClosed();
	}
	
	// Check Latch catapult limit switch
	public boolean checkLatchLimit() {
		return m_resetBar.isRevLimitSwitchClosed();
	} 
	
	// PIDController methods
	protected double returnPIDInput() {
		int angle = m_resetBar.getAnalogInRaw();
		if (debugging) {
	    	SmartDashboard.putBoolean("Shooter PID Enabled", true);
	    	SmartDashboard.putNumber("Shooter Reset Angle", angle);
		}
		return angle;
	}

	protected void usePIDOutput(double output) {
		if (debugging) {
			SmartDashboard.putNumber("Shooter Setpoint", this.getSetpoint());
			SmartDashboard.putNumber("Shooter Current",
					Brownout.getInstance().getCurrent(WINCH_PDP_CHANNEL));			
		}
		m_resetBar.set(check4Endpoints(output));
	}

	private double check4Endpoints(double speed) {
		SmartDashboard.putBoolean("Shooter Out of Calibration", false);
		// If trying to drive and a limit switch is hit, then stop...
		if(checkClearLimit() && speed > 0.0) {
			speed = 0.0;
			if (Math.abs(m_resetBar.getAnalogInRaw() - m_clearPoint) > 200)
				SmartDashboard.putBoolean("Shooter Out of Calibration", true);
		}
		else if(checkLatchLimit() && speed < 0.0) {
			speed = 0.0;
			if (Math.abs(m_resetBar.getAnalogInRaw() - m_latchPoint) > 200)
				SmartDashboard.putBoolean("Shooter Out of Calibration", true);
		}
		return(speed);
	}


	public void resetMP() {
		shooterTalon.clearMotionProfileTrajectories();
		shooterTalon.disableMotionProfiling();
		
	}

	public void startMP(BuildTrajectory trajectory) {
		shooterTalon.startFilling(trajectory.getprofile(), trajectory.getTotalCount(), false);
		
		shooterTalon.changeMotionControlFramePeriod(20);
		
		shooterTalon.enableMotionProfiling();
	}

	public void startMPwithAddition(BuildTrajectory trajectory, boolean invert, int addition) {
		shooterTalon.startFillingPlusSome(trajectory.getprofile(), trajectory.getTotalCount(), invert, addition);
		
		shooterTalon.changeMotionControlFramePeriod(20);
		
		shooterTalon.enableMotionProfiling();
	}
	
	
	public void publishValues() {
		SmartDashboard.putNumber("Shooter top buffer", shooterTalon.topBuffercount());
		SmartDashboard.putNumber("Shooter bottom buffer", shooterTalon.BottomBufferCount());
		SmartDashboard.putBoolean("ShooterMP is Complete", shooterTalon.isComplete());
		
		SmartDashboard.putBoolean("Shooter is underrun", shooterTalon.isUnderrun());
		SmartDashboard.putBoolean("Shooter has underrun", shooterTalon.hasUnderrun());
	}

	public void processMotionProfileBuffer() {
		shooterTalon.processMotionProfileBuffer();
	}

	public void updateMotionProfileStatus() {
		shooterTalon.upDateMotionProfileStatus();
	}

	public boolean isComplete() {
		if(shooterTalon.isComplete()) {
			return true;
		}
		else {
			return false;
		}
	}
}
