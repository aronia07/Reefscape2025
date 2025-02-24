package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;

public class GroupAnimationSequence extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;
    private final String groupID;
    private final String sequenceID;
    public GroupAnimationSequence(LEDSubsystem_LuminLabs subsystem, String groupID, String sequenceID) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.sequenceID = sequenceID;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupAnimationSequence(groupID, sequenceID);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}