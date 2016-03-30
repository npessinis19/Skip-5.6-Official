package org.usfirst.frc.team3467.robot.subsystems.compressor;

import org.usfirst.frc.team3467.robot.RobotMap;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.subsystems.Brownout.PowerConsumer;
import org.usfirst.frc.team3467.robot.subsystems.compressor.commands.Compressor_Update;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pneumatics extends Subsystem implements PowerConsumer {

	private Compressor scorpionCompressor;
	private AnalogInput pressureSensor;
	private boolean compressorActive = false;
	
	// Pneumatics is a singleton
	private static Pneumatics instance = new Pneumatics();

	public static Pneumatics getInstance() {
		return Pneumatics.instance;
	}

	/*
	 * Pneumatics Class Constructor
	 *
	 * The singleton instance is created statically with
	 * the instance static member variable.
	 */
	protected Pneumatics() {
		instance = this;
		
		scorpionCompressor = new Compressor();
		pressureSensor = new AnalogInput(RobotMap.pneumatics_sensor_port);
		scorpionCompressor.stop();;
		compressorActive = true;
		
		Brownout.getInstance().registerCallback(this);
	}
	
	public double getPressure() {
		return pressureSensor.getVoltage();
	}
	
	public void compressorStop() {
		scorpionCompressor.stop();
		compressorActive = false;
	}
	
	public void compressorStart() {
		scorpionCompressor.start();
		compressorActive = true;
	}
	
	public void callbackAlert(Brownout.PowerLevel level) {
		switch (level) {
		case Critical:
			if (compressorActive) {
				scorpionCompressor.stop();
				compressorActive = false;
			}
			break;
		
		case Normal:
		default:
			if (!compressorActive) {
				//scorpionCompressor.start();
				compressorActive = true;
			}
			break;
		}
	}

	// Set up a default command to regularly call reportPressure()
	protected void initDefaultCommand() {
		this.setDefaultCommand(new Compressor_Update());
	}

	public void reportPressure() {
		SmartDashboard.putBoolean("Compressor Active?", compressorActive);
		SmartDashboard.putNumber("Pressure (voltage)", pressureSensor.getVoltage());		
	}
}
