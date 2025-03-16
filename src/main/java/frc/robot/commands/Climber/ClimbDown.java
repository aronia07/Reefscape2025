package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class ClimbDown extends Command {
    protected final Climber climber_y;
    private DoubleSupplier powerSupplier;

    public ClimbDown(Climber climber, DoubleSupplier doubleSupplier) {
        this.climber_y = climber;
        this.powerSupplier = doubleSupplier;
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();
        climber_y.down(power);
    }

    @Override
    public void end(boolean interrupted) {
        climber_y.down(0);
    }

}
