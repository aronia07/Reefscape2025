package frc.robot.commands.Elevator;

import java.lang.annotation.ElementType;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateLevel extends Command {
    protected final Elevator elevator_y;
    private State initialState;
    private IntSupplier levelSupplier;
    private TrapezoidProfile profiler_y = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAccel));
    private Timer timer_y = new Timer();

    public ElevateLevel(Elevator elevator, IntSupplier doubleSupplier) {
        elevator_y = elevator;
        this.levelSupplier = doubleSupplier;

    }

    @Override
    public void initialize() {
        timer_y.reset();
        timer_y.start();

        int level = levelSupplier.getAsInt();
        switch(level){
        case 1:
            elevator_y.setMode(ElevateMode.L1);
            break;
        case 2:
            elevator_y.setMode(ElevateMode.L2);
            break;
        case 3:
            elevator_y.setMode(ElevateMode.L3);
            break;
        case 4:
            elevator_y.setMode(ElevateMode.L4);
            break;
        case 5:
            elevator_y.setMode(ElevateMode.DOWN);
            break;
        case 6:
            elevator_y.setMode(ElevateMode.UP);
            break;
        case 7:
            elevator_y.setMode(ElevateMode.TEST);
            break;
        default:
            break;
        }
        // if (level == 4) {
        // elevator_y.setMode(ElevateMode.L4);
        // } if (level == 3) {
        // elevator_y.setMode(ElevateMode.L3);
        // } if (level == 2) {
        // elevator_y.setMode(ElevateMode.L2);
        // } if (level == 1) {
        // elevator_y.setMode(ElevateMode.L1);
        // } if (level == 0) {
        // elevator_y.setMode(ElevateMode.HP);
        // } if (level == 5) {
        // elevator_y.setMode(ElevateMode.DOWN);
        // } if (level == 6) {
        // elevator_y.setMode(ElevateMode.UP);
        // } if (level == 7) {
        // elevator_y.setMode(ElevateMode.TEST);
        // } if (level == 8) {
        //     elevator_y.setMode(ElevateMode.MANUAL);
        // }

        initialState = elevator_y.getCurrentState();
    }

    @Override
    public void execute() {
        
        var nextState = profiler_y.calculate(timer_y.get(),
                initialState,
                new TrapezoidProfile.State(elevator_y.getSetpoint(), 0));

        // var nextNextState = profiler_y.calculate(timer_y.get() + 0.02,
        //         initialState,
        //         new TrapezoidProfile.State(elevator_y.getSetpoint(), 0));

        // elevator_y.runState(nextState, nextNextState);
        elevator_y.runState(nextState);
    }

    @Override
    public void end(boolean interrupted) {
        timer_y.stop();
        elevator_y.setMode(ElevateMode.OFF);
    }

}
