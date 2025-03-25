package frc.robot.commands.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateManual extends Command {
    protected final Elevator elevator_y;
    private BooleanSupplier booleanSupplier;
    private boolean up;
    
    public ElevateManual(BooleanSupplier direction, Elevator elevator){
        elevator_y = elevator;
        this.booleanSupplier = direction;
        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        this.up = booleanSupplier.getAsBoolean();

    }

    @Override
    public void execute() {
        if(up){
            elevator_y.leftElevatorMotor.set(0.1);
            elevator_y.rightElevatorMotor.set(-.1);
        } else {
            elevator_y.leftElevatorMotor.set(-.1);
            elevator_y.rightElevatorMotor.set(.1);
        }

        // // value of joystick
        // double elevatorStick = MathUtil.applyDeadband(this.doubleSupplier.getAsDouble(), 0.1); 
        // // sensitivity of joystick
        // double elevatorChange = (50/50)*elevatorStick;

        // this.elevator_y.setSetpoint(elevatorChange + this.elevator_y.getSetpoint());
    }
    @Override
    public void end(boolean interrupted) {
        elevator_y.leftElevatorMotor.set(0);
        elevator_y.rightElevatorMotor.set(0);
    }
    
}
