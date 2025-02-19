package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class SetSolidColor extends Command {
    private final LEDSubsystem ledSubsystem;
    private final Color color;

    public SetSolidColor(LEDSubsystem subsystem, Color color) {
        this.ledSubsystem = subsystem;
        this.color = color;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_SolidColor(color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}