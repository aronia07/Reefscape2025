package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;

public class ZoneAnimationSequence extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;
    private final String zoneID;
    private final String sequenceID;
    public ZoneAnimationSequence(LEDSubsystem_LuminLabs subsystem, String zoneID, String sequenceID) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.sequenceID = sequenceID;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupAnimationSequence(zoneID, sequenceID);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}