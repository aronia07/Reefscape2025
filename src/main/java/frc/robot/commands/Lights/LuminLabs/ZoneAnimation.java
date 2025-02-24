package frc.robot.commands.Lights.LuminLabs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_LuminLabs;
import com.lumynlabs.domain.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

public class ZoneAnimation extends Command {
    private final LEDSubsystem_LuminLabs ledSubsystem;
    private final String zoneID;
    private final Animation animation;
    private final Color color;
    private final int delay;
    private final boolean reversed;
    private final boolean oneShot;

    public ZoneAnimation(LEDSubsystem_LuminLabs subsystem, String zoneID, Animation animation, Color color, int delay, boolean reversed, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.animation = animation;
        this.color = color;
        this.delay = delay;
        this.reversed = reversed;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_ZoneAnimation(zoneID, animation, color, delay, reversed, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}