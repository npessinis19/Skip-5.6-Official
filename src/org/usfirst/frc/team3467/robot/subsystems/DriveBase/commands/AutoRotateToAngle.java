package org.usfirst.frc.team3467.robot.subsystems.DriveBase.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turn the robot to a given heading (in degrees relative to last reset heading;
 * negative values go counterclockwise).
 * Uses a local PID controller to run a simple PID loop that is only
 * enabled while this command is running. The input is the yaw() value from AHRS.
 */
public class AutoRotateToAngle extends CommandBase {

	private static final double TOLERANCE = 0.5;
	
	private PIDController m_pid;
	private double m_maxSpeed = 0.3;
	private double m_degrees = 0.0;
	
	private double KP = 2.0;
	private double KI = 0.5;
	private double KD = 1.0;
	
	double e_min = 1000;
	boolean stop = false;
	int count = 0;
	private static final int sufficientCount = 100;
    	
    public AutoRotateToAngle(double degrees, double maxSpeed, double kp, double ki, double kd) {
                
    	requires(driveBase);
    	KP = kp; KI = ki; KD = kd;
    	m_maxSpeed = maxSpeed;
    	m_degrees = degrees;
    	buildController();
    //setTimeout(10);
    }

	public AutoRotateToAngle(double degrees, double maxSpeed) {
                
		requires(driveBase);
    	m_maxSpeed = maxSpeed;
    	m_degrees = degrees;
    	buildController();
	}
    	
	public AutoRotateToAngle(double degrees) {
    	        
    	requires(driveBase);
    	m_degrees = degrees;
    	buildController();
	}
    	
    private void buildController() {
 
       m_pid = new PIDController(KP, KI, KD,
                new PIDSource() {
                    PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

                    public double pidGet() {
                        return ahrs.getGyroYaw();
                    }

                    public void setPIDSourceType(PIDSourceType pidSource) {
                      m_sourceType = pidSource;
                    }

                    public PIDSourceType getPIDSourceType() {
                        return m_sourceType;
                    }
                },
                new PIDOutput() {
                	
                	public void pidWrite(double d) {
                		// Spin with the magnitude returned by the PID calculation,
                		double er = Math.abs(m_pid.getError());

                    	if (er >= 0 && er <= TOLERANCE) {
                    		m_pid.setPID(KP - 0.068 * (count+1), KI, KD);
                    		 if (count++ > sufficientCount) {
                    			 stop = true;
                    		 } 
                    	}
                    	else {
                    	count = 0;
                		driveBase.driveArcade(0, d, false);
                    	}
                }});
		m_pid.setAbsoluteTolerance(TOLERANCE);
		m_pid.setOutputRange((m_maxSpeed * -1.0), m_maxSpeed);
        m_pid.setSetpoint(m_degrees);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	SmartDashboard.putNumber("P", KP);
    	SmartDashboard.putNumber("I", KI);
    	SmartDashboard.putNumber("D", KD);
    	
    	stop = false;
    	count = 0;
    	
    	//Brake mode on
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
		
    	// Get everything in a safe starting state.
    	m_pid.reset();
        m_pid.enable();
        SmartDashboard.putNumber("Rotate SetPoint", m_pid.getSetpoint());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Gyro Angle", ahrs.getGyroAngle());
    	
    	double p = SmartDashboard.getNumber("P");
    	double i = SmartDashboard.getNumber("I");
    	double d = SmartDashboard.getNumber("D");
    	//m_pid.setPID(m_pid.getP(), i, d);
    	//m_pid.setPID(p, i, d);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return stop;
    }
    

    // Called once after isFinished returns true
    protected void end() {
    	// Stop PID and the wheels
    	m_pid.disable();
        driveBase.driveArcade(0, 0, false);
        System.out.println("AutoRotate has finished");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
