package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;
import edu.wpi.first.wpilibj.util.Color;

public class ZoneImageSequence extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;
    private final String zoneID;
    private final String sequenceID;
    private final Color color;
    private final boolean setColor;
    private final boolean oneShot;

    public ZoneImageSequence(LEDSubsystem_LuminLabs subsystem, String zoneID, String sequenceID, Color color, boolean setColor, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.sequenceID = sequenceID;
        this.color = color;
        this.setColor = setColor;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_ZoneImageSequence(zoneID, sequenceID, color, setColor, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}