package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;

public class Climb extends Command {
    protected final Climber climber_y;
    private DoubleSupplier powerSupplier;

    public Climb(Climber climber, DoubleSupplier doubleSupplier){
        this.climber_y = climber;
        this.powerSupplier = doubleSupplier;
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();
        climber_y.go(power);
    }
    
}
