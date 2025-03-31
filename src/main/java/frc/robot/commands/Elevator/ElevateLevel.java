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
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateLevel extends Command {
    protected final Elevator elevator_y;
    // private double lastSpeed;
    // private double lastTime;
    // private State initialState;
    private ProfiledPIDController elevatorPID;
    private ElevateMode level;
    // private TrapezoidProfile profiler_y = new TrapezoidProfile(
    // new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity,
    // ElevatorConstants.maxAccel));
    // private Timer timer_y = new Timer();

    public ElevateLevel(Elevator elevator, ElevateMode mode) {
        this.elevator_y = elevator;
        this.level = mode;

        addRequirements(elevator_y);
    }

    @Override
    public void initialize() {
        // lastSpeed = 0;
        // lastTime = Timer.getFPGATimestamp();
        elevatorPID = elevator_y.pid;
        elevatorPID.setTolerance(.3);

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
            case L3AR:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelThreeAR;
                elevatorPID.setGoal(ElevatorConstants.LevelThreeAR);
                break;
            case L3M:
                elevator_y.elevatorSetpoint = ElevatorConstants.LevelThreeSetpointM;
                elevatorPID.setGoal(ElevatorConstants.LevelThreeSetpointM);
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
            default:
                break;
        }

    }

    @Override
    public void execute() {
        // double acceleration = (elevatorPID.getSetpoint().velocity - lastSpeed) /
        // (Timer.getFPGATimestamp() - lastTime);

        // var ffoutput =
        // Math.sin(Arm.getEncoderPosition().getRadians())*ElevatorConstants.elevatorSGV[1]
        // +
        // elevator_y.ffElevate.calculate(elevatorPID.getSetpoint().velocity/*,
        // acceleration*/);

        // var pidOutput = elevatorPID.calculate(elevator_y.encoderPosition);

        // elevator_y.leftElevatorMotor.set(-pidOutput - ffoutput);
        // elevator_y.rightElevatorMotor.set(pidOutput + ffoutput);

        // lastSpeed = elevatorPID.getSetpoint().velocity;
        // lastTime = Timer.getFPGATimestamp();
        // if(level == ElevateMode.HOMING){
        // elevator_y.leftElevatorMotor.set(.05);
        // elevator_y.rightElevatorMotor.set(-.05);
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
    // @Override
    // public boolean isFinished() {
    // return elevatorPID.atSetpoint();
    // }

    @Override
    public void end(boolean interrupted) {
        // timer_y.stop();
        // elevator_y.elevatorSetpoint = 1;
        // elevator_y.leftElevatorMotor.set(0);
        // elevator_y.rightElevatorMotor.set(0);
    }

}
