package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;

public class ReselLED_OLD extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;

    public ReselLED_OLD(LEDSubsystem_WPIlib subsystem) {
        this.ledSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}