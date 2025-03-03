package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;
import edu.wpi.first.wpilibj.util.Color;

public class ZoneSolidColor extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;
    private final String zoneID;
    private final Color color;

    public ZoneSolidColor(LEDSubsystem_LuminLabs subsystem, String zoneID, Color color) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.color = color;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_ZoneColor(zoneID, color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}