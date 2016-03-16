package org.usfirst.frc.team3467.robot.subsystems.compressor;

import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout.PowerLevel;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.PowerConsumer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Pneumatics extends Subsystem implements PowerConsumer {

	public Compressor scorpionCompressor;
	public AnalogInput pressureSwitch;
	
	private Pneumatics instance;
	
	public Pneumatics getInstance() {
		return instance;
	}
	
	public Pneumatics() {
		instance = this;
		
		scorpionCompressor = new Compressor();
		pressureSwitch = new AnalogInput(0);
		scorpionCompressor.start();
	}
	
	public double getPressure() {
		return pressureSwitch.getVoltage();
	}
	
	protected void initDefaultCommand() {
	}

	public void callbackAlert(Brownout.PowerLevel level) {
		switch (level) {
		case Normal:
								scorpionCompressor.start();
			break;
		case Critical:
								scorpionCompressor.stop();
			break;
		default:
								scorpionCompressor.start();
			break;
		}
	}
}
