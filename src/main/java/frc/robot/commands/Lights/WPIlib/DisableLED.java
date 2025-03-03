package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;

public class DisableLED extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;

    public DisableLED(LEDSubsystem_WPIlib subsystem) {
        this.ledSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Disable();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
