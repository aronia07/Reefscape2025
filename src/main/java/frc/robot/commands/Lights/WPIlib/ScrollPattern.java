package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import edu.wpi.first.wpilibj.LEDPattern;

public class ScrollPattern extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final LEDPattern pattern;
    private final double magnitude;

    public ScrollPattern(LEDSubsystem_WPIlib subsystem, LEDPattern pattern, double magnitude) {
        this.ledSubsystem = subsystem;
        this.pattern = pattern;
        this.magnitude = magnitude;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_ScrollPatternRelative(pattern, magnitude);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}