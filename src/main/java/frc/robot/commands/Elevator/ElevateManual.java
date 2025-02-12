package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateManual extends Command {
    protected final Elevator elevator_y;
    private DoubleSupplier doubleSupplier;
    
    public ElevateManual(Elevator elevator, DoubleSupplier doubleSupplier){
        elevator_y = elevator;
        this.doubleSupplier = doubleSupplier;
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void end(boolean interrupted){
        elevator_y.setMode(ElevateMode.OFF);
    }
    
}
