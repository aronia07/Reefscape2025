package frc.robot.commands.Elevator;

import java.lang.annotation.ElementType;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private ProfiledPIDController elevatorPID;
    // private TrapezoidProfile profiler_y = new TrapezoidProfile(
    // new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity,
    // ElevatorConstants.maxAccel));
    // private Timer timer_y = new Timer();

    public ElevateLevel(Elevator elevator, ElevateMode mode) {
        elevator_y = elevator;
        this.level = mode;

        addRequirements(elevator_y);
    }

    @Override
    public void initialize() {
        elevatorPID = elevator_y.pid;

        switch (level) {
            case UP:
                break;
            case DOWN:
                elevator_y.elevatorSetpoint = 1;
                elevatorPID.setGoal(1);
                break;
            case L1:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
                elevatorPID.setGoal(ElevatorConstants.LevelOneSetpoint);
                break;
            case L2:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelTwoSetpoint;
                elevatorPID.setGoal(ElevatorConstants.LevelTwoSetpoint);
                break;
            case L2AR:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelTwoAlgaeSetpoint;
                elevatorPID.setGoal(ElevatorConstants.LevelTwoAlgaeSetpoint);
                break;
            case L3:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelThreeSetpoint;
                elevatorPID.setGoal(ElevatorConstants.LevelThreeSetpoint);
                break;
            case L4:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
                elevatorPID.setGoal(ElevatorConstants.LevelFourSetpoint);
                break;
            case HP:
                elevator_y.elevatorSetpoint = ElevatorConstants.HPsetpoint;
                elevatorPID.setGoal(ElevatorConstants.HPsetpoint);
                break;
            case TEST:
                elevator_y.elevatorSetpoint = ElevatorConstants.test;
                elevatorPID.setGoal(ElevatorConstants.test);
                break;
            case OFF:
                elevator_y.elevatorSetpoint = 1;
                elevatorPID.setGoal(1);
                break;
            case RESET:
                elevator_y.SET(0);
                elevator_y.reset();
                break;
            case HOMING:
                elevator_y.elevatorSetpoint = .5;
                elevatorPID.setGoal(.5);
                // if (elevator_y.leftElevatorMotor.getOutputCurrent() >= ElevatorConstants.homingCurrentThreshold) {
                //     elevator_y.leftElevatorMotor.setVoltage(0);
                //     elevator_y.rightElevatorMotor.setVoltage(0);
                //     elevator_y.SET(0);
                    
                // } else {
                //     elevator_y.leftElevatorMotor.setVoltage(ElevatorConstants.selfHomeSpeedVoltage);
                //     elevator_y.rightElevatorMotor.setVoltage(ElevatorConstants.selfHomeSpeedVoltage);
                // }
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
        var ffoutput = elevator_y.ffElevate.calculate(elevatorPID.getSetpoint().velocity);
        var pidOutput = elevatorPID.calculate(elevator_y.encoderPosition);
        elevator_y.leftElevatorMotor.set(-pidOutput - ffoutput);
        elevator_y.rightElevatorMotor.set(pidOutput + ffoutput);
        // if(level == ElevateMode.HOMING){
        //     elevator_y.leftElevatorMotor.set(.05);
        //     elevator_y.rightElevatorMotor.set(-.05);
        // }
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
        // timer_y.stop();
        // elevator_y.elevatorSetpoint = 1;
        // elevator_y.leftElevatorMotor.set(0);
        // elevator_y.rightElevatorMotor.set(0);
    }

}
