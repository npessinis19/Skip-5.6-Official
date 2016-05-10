package org.usfirst.frc.team3467.robot.subsystems.Vision.commands;

import org.usfirst.frc.team3467.robot.commands.CommandBase;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAim extends CommandBase {
	
	private static final double TOLERANCE = 0.5;
	
	private PIDController m_pid;
	
	private double m_maxSpeed = 0.35;
	
	//Initial PID constants
	private double KP = 2.0;
	private double KI = 0.05;
	private double KD = 0.3;
	
	//Management Variables
	private boolean stop = false; //Used to turn off command if robot in tolerance zone
	private int count = 0; //Counter starts when robot is in tolerance zone
	private static final int sufficientCount = 28; //When count is this number, the command will end
	private double oldKP = KP;
	private int aimState = 1; //Is the robot "moving" (Setpoint has been set
	private int imageCount = 0; 
	
	private static final boolean debugging = false;

    public AutoAim(double maxSpeed, double kp, double ki, double kd) {    
    	requires(driveBase);
    	KP = kp; KI = ki; KD = kd;
    	m_maxSpeed = maxSpeed;
    	buildController();
    //setTimeout(10);
    }

	public AutoAim(double maxSpeed) {
		requires(driveBase);
    	m_maxSpeed = maxSpeed;
    	buildController();
	}
    	
	public AutoAim() {
    	requires(driveBase);
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
                		double er = m_pid.getError();

                    	if (Math.abs(er) >= 0 && Math.abs(er) <= TOLERANCE) {
                    		if (KP > 0.05) {
                    			KP = oldKP - 0.3 * (count+1);
                    			
                    			if (KP < 0.01) KP = 0.01;
                    			
                    			m_pid.setPID(KP, KI, KD);
                    			oldKP = KP;
                    		}

                    		System.out.println("KP " + KP);
                    		System.out.println("Count " + count);
                    		
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
        //m_pid.setSetpoint(m_degrees);
    }

	public void deBug() {
		if (debugging) {	
			double p = SmartDashboard.getNumber("P");
			double i = SmartDashboard.getNumber("I");
			double d = SmartDashboard.getNumber("D");
    	
			m_pid.setPID(m_pid.getP(), i, d);
    	
			SmartDashboard.putNumber("P", KP);
		}
	}
    
    
	protected void initialize() {
    	SmartDashboard.putNumber("P", KP);
    	SmartDashboard.putNumber("I", KI);
    	SmartDashboard.putNumber("D", KD);
    	
    	//Initialize management
    	stop = false;
    	count = 0;
    	oldKP = KP;
    	imageCount = 0;
    	aimState = 1;
    	
    	//Brake mode on
		driveBase.setSlaveMode(false);
		driveBase.setTalonBrakes(true);
		
    	// Get everything in a safe starting state.
    	ahrs.gyroReset();
		m_pid.reset();
        m_pid.enable();
        SmartDashboard.putNumber("Rotate SetPoint", m_pid.getSetpoint());
	}

	protected void execute() {
		switch (aimState) {
		//State one, search for a good image
		case 1: if(!grip.isGoodImage()) {
					grip.createImage();
				}
				else {
					aimState = 2;
				}
			break;
		//Calculate target Data
		case 2:
				grip.calculateTargetData();
				aimState = 3;
			break;
		//Move to correct position
		case 3:
				m_pid.setSetpoint(grip.getChangeinAngle());
				aimState = 4;
			break;
		//Check if we're on Target (if not, go to aimState 1)
		case 4: 
				if (!grip.isOnTarget()) {
					aimState = 1;
				}
				else {
					System.out.println("AutokAim on Target");
					end();
				}
			break;
		}
		
		SmartDashboard.putNumber("Vision: aimState", aimState);
    	SmartDashboard.putNumber("Gyro Angle", ahrs.getGyroAngle());
    	SmartDashboard.putNumber("Error", m_pid.getError());
    	deBug();
	}

	protected boolean isFinished() {
		return stop;
	}

	protected void end() {
    	// Stop PID and the wheels
    	m_pid.reset();
        driveBase.driveArcade(0, 0, false);
        System.out.println("AutoAim has finished");
	}

	protected void interrupted() {
		end();
	}
}
