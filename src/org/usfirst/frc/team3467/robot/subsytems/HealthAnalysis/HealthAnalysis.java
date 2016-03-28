package org.usfirst.frc.team3467.robot.subsytems.HealthAnalysis;

import org.usfirst.frc.team3467.robot.subsystems.Brownout.Brownout;
import org.usfirst.frc.team3467.robot.commands.CommandBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class HealthAnalysis {
private PowerDistributionPanel health = CommandBase.brownout.getpdp();
}
