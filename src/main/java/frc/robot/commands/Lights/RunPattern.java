package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import edu.wpi.first.wpilibj.LEDPattern;

public class RunPattern extends Command {
    private final LEDSubsystem ledSubsystem;
    private final LEDPattern pattern;
    private final boolean animated;

    public RunPattern(LEDSubsystem subsystem, LEDPattern pattern, boolean animated) {
        this.ledSubsystem = subsystem;
        this.pattern = pattern;
        this.animated = animated;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.runPattern(pattern, animated);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}