package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.Wrist;

public class WristMove extends Command {
    private  Wrist wrist_y;
    private DoubleSupplier wristSupplier;

    public WristMove(DoubleSupplier doubleSupplier, Wrist wrist) {
        this.wrist_y = wrist;
        this.wristSupplier = doubleSupplier;
        addRequirements(wrist);

    }    

    @Override
    public void initialize() {
 
    }

    @Override
    public void execute() {
        double wristY = MathUtil.applyDeadband(wristSupplier.getAsDouble(), 0.1);

        double wristChange =  wristY;

    this.wrist_y.runSetpoint(new Rotation2d(
    Units.degreesToRadians(wristChange + wrist_y.getSetpoint().getDegrees())));

    
    }
    
}
