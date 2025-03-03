package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import edu.wpi.first.wpilibj.LEDPattern;

public class SetBlinkingPattern extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final LEDPattern pattern;
    private final double onTime;
    private final double offTime;

    public SetBlinkingPattern(LEDSubsystem_WPIlib subsystem, LEDPattern pattern, double onTime, double offTime) {
        this.ledSubsystem = subsystem;
        this.pattern = pattern;
        this.onTime = onTime;
        this.offTime = offTime;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Blinking(pattern, onTime, offTime);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}