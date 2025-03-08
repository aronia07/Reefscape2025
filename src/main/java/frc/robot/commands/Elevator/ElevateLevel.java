package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateLevel extends Command {
    protected final Elevator elevator_y;
    // private State initialState;
    private ElevateMode level;
    // private TrapezoidProfile profiler_y = new TrapezoidProfile(
    //         new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAccel));
    private Timer timer_y = new Timer();

    public ElevateLevel(Elevator elevator, ElevateMode mode) {
        elevator_y = elevator;
        this.level = mode;

        addRequirements(elevator_y);
    }

    @Override
    public void initialize() {
        timer_y.reset();
        timer_y.start();

        switch (level) {
            case UP:
                break;
            case DOWN:
                elevator_y.elevatorSetpoint = 1;
                break;
            case L1:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
                break;
            case L2:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelTwoSetpoint;
                break;
            case L3:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelThreeSetpoint;
                break;
            case L4:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
                break;
            case HP:
                elevator_y.elevatorSetpoint = ElevatorConstants.HPsetpoint;
                break;
            case TEST:
                elevator_y.elevatorSetpoint = ElevatorConstants.test;
                break;
            case OFF:
                elevator_y.elevatorSetpoint = 1;
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
        // elevator_y.setMode(ElevateMode.MANUAL);
        // }

        // initialState = elevator_y.getCurrentState();
    }

    @Override
    public void execute() {

        // var nextState = profiler_y.calculate(timer_y.get(),
        // initialState,
        // new TrapezoidProfile.State(elevator_y.getSetpoint(), 0));

        // var nextNextState = profiler_y.calculate(timer_y.get() + 0.02,
        // initialState,
        // new TrapezoidProfile.State(elevator_y.getSetpoint(), 0));

        // elevator_y.runState(nextState, nextNextState);
        // elevator_y.runState(nextState);
    }

    @Override
    public void end(boolean interrupted) {
        timer_y.stop();
        elevator_y.elevatorSetpoint = 1;
    }

}
