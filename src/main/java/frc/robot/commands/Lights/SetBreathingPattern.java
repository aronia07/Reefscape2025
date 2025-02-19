package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import edu.wpi.first.wpilibj.LEDPattern;

public class SetBreathingPattern extends Command {
    private final LEDSubsystem ledSubsystem;
    private final LEDPattern pattern;
    private final double period;

    public SetBreathingPattern(LEDSubsystem subsystem, LEDPattern pattern, double period) {
        this.ledSubsystem = subsystem;
        this.pattern = pattern;
        this.period = period;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Breathing(pattern, period);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}