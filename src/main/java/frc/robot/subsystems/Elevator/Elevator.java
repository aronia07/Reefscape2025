package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.commands.Elevator.ElevateLevel;

public class Elevator extends SubsystemBase {

  private SparkMax leftElevatorMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
  private SparkMax rightElevatorMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  private PIDController pid = new PIDController(ElevatorConstants.elevatorPID[0], ElevatorConstants.elevatorPID[1],
      ElevatorConstants.elevatorPID[2]);
  private ElevatorFeedforward ffElevate = new ElevatorFeedforward(ElevatorConstants.elevatorSGV[0],
      ElevatorConstants.elevatorSGV[1], ElevatorConstants.elevatorSGV[2], ElevatorConstants.elevatorSGV[3]);

  private final RelativeEncoder encoderLeft;
  // private final RelativeEncoder encoderRight;

  private Timer timer = new Timer();
  public double encoderPosition;
  private double nextVelocity = 0.0;
  private double nextNextVelocity = 0.0;
  public double elevatorSetpoint = 0.05;
  private double positionRateOfChange = 0;
  private double changingSetpoint = 0;

  // private double leftPower = 0;
  // private double rightPower = 0;
  private ElevateMode elevateMode = ElevateMode.OFF;
  private boolean isLeftDone = false;
  // private boolean isRightDone = false;

  /* Tunable Values */
  private LoggedTunableNumber elevatorP = new LoggedTunableNumber("elevatorP", ElevatorConstants.elevatorPID[0]);
  private LoggedTunableNumber elevatorI = new LoggedTunableNumber("elevatorI", ElevatorConstants.elevatorPID[1]);
  private LoggedTunableNumber elevatorD = new LoggedTunableNumber("elevatorD", ElevatorConstants.elevatorPID[2]);
  private LoggedTunableNumber elevatorS = new LoggedTunableNumber("elevatorS", ElevatorConstants.elevatorSGV[0]);
  private LoggedTunableNumber elevatorG = new LoggedTunableNumber("elevatorG", ElevatorConstants.elevatorSGV[1]);
  private LoggedTunableNumber elevatorV = new LoggedTunableNumber("elevatorV", ElevatorConstants.elevatorSGV[2]);
  private LoggedTunableNumber elevatorA = new LoggedTunableNumber("elevatorA", ElevatorConstants.elevatorSGV[3]);
  private LoggedTunableNumber elevatorLevel = new LoggedTunableNumber("elevator setpoint", elevatorSetpoint);

  public Elevator() {
    setupMotors();
    encoderLeft = rightElevatorMotor.getEncoder();
    // encoderRight = leftElevatorMotor.getEncoder();
    resetEncoders();

  }

  private static enum States {
    BADBADBAD,
    OKAYUP,
    OKAYDOWN,
    GOOD
  }

  private void setupMotors() {
    /* Motor Setup */
    leftConfig.inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .voltageCompensation(12);

    rightConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 40)
        .voltageCompensation(12);

    leftElevatorMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightElevatorMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  public void setMode(ElevateMode mode) {
    elevateMode = mode;
  }

  public void setSetpoint(double goal) {
    this.elevatorSetpoint = goal;
  }

  public double getSetpoint() {
    return this.elevatorSetpoint;
  }

  public void SET(double value) {
    encoderLeft.setPosition(value);
  }

  public double ActualPositionLeader() {
    return encoderLeft.getPosition();
  }

  public double getDesiredPositionLeader() {
    return positionRateOfChange;
  }

  private void resetEncoders() {
    encoderLeft.setPosition(0);
    // encoderRight.setPosition(0);
  }

  public void logValues() {
    SmartDashboard.putNumber("Actual Elevator Position Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Desired Elevator Setpoint", elevatorSetpoint);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(encoderPosition, nextVelocity);
  }

  public void runState(TrapezoidProfile.State state, TrapezoidProfile.State newState) {
    this.elevatorSetpoint = state.position;
    this.nextVelocity = state.velocity;
    this.nextNextVelocity = newState.velocity;
  }

  public void checkTunableValues() {
    if (Constants.enableTunableValues) {

      if (elevatorP.hasChanged() || elevatorI.hasChanged() || elevatorD.hasChanged()) {
        pid.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());
        leftConfig.closedLoop.pid(elevatorP.get(), elevatorI.get(), elevatorD.get());
      }
      if (elevatorS.hasChanged() || elevatorG.hasChanged() || elevatorV.hasChanged() || elevatorA.hasChanged()) {
        ffElevate = new ElevatorFeedforward(elevatorS.get(), elevatorG.get(), elevatorV.get(), elevatorA.get());
      }
    }
  }

  // public boolean isDone() {
  // return isLeftDone && isRightDone;
  // }

  public States outOfBounds(double encoderValue) {
    if (encoderValue <= ElevatorConstants.min) {
      return States.BADBADBAD;
    } else if (encoderValue <= ElevatorConstants.desiredMin) {
      return States.OKAYUP;
    } else if (encoderValue < ElevatorConstants.desiredMax) {
      return States.GOOD;
    } else if (encoderValue < ElevatorConstants.max) {
      return States.OKAYDOWN;
    } else if (encoderValue >= ElevatorConstants.max) {
      return States.BADBADBAD;
    } else {
      return States.BADBADBAD;
    }
  }

  // public States isRightOutOfBounds() {
  // return outOfBounds(-encoderRight.getPosition());
  // }

  public States isLeftOutOfBounds() {
    return outOfBounds(encoderLeft.getPosition());
  }

  /**
   * Logic for the elevator command, switches through modes to
   * control elevator
   */
  private void handleLeft() {
    // case HOMING:
    // if (leftClimberMotor.getOutputCurrent() >=
    // ClimberConstants.homingCurrentThreshold) {
    // if (timer.get() < 1)
    // return;
    // stopLeft();
    // isLeftDone = true;
    // encoderLeft.setPosition(0);
    // } else {
    // setLeft(ClimberConstants.selfHomeSpeedVoltage);
    // }
    // break;
    switch (elevateMode) {
      case UP:
        break;
      case DOWN:
        elevatorSetpoint = 2;
        break;
      case L1:
        elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
        break;
      case L2:
        elevatorSetpoint = ElevatorConstants.LevelTwoSetpoint;
        break;
      case L3:
        elevatorSetpoint = ElevatorConstants.LevelThreeSetpoint;
        break;
      case L4:
        elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
        break;
      case HP:
        elevatorSetpoint = ElevatorConstants.HPsetpoint;
        break;
      case MANUAL:
        this.elevatorSetpoint = elevatorLevel.get();
        break;
      case TEST:
        elevatorSetpoint = ElevatorConstants.test;
        break;
      case OFF:
        elevatorSetpoint = 1;
        break;
      case HOMING:
        if (leftElevatorMotor.getOutputCurrent() >= ElevatorConstants.homingCurrentThreshold) {
          if (timer.get() < 1)
            return;
          leftElevatorMotor.setVoltage(0);
          isLeftDone = true;
          encoderLeft.setPosition(0);
        } else {
          leftElevatorMotor.setVoltage(ElevatorConstants.selfHomeSpeedVoltage);
        }
        break;
      default:
        break;
    }
  }

  private void handleRight() {
    // case HOMING:
    // if (leftClimberMotor.getOutputCurrent() >=
    // ClimberConstants.homingCurrentThreshold) {
    // if (timer.get() < 1)
    // return;
    // stopLeft();
    // isLeftDone = true;
    // encoderLeft.setPosition(0);
    // } else {
    // setLeft(ClimberConstants.selfHomeSpeedVoltage);
    // }
    // break;
    switch (elevateMode) {
      case UP:
        break;
      case DOWN:
        break;
      case L1:
        elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
        break;
      case L2:
        elevatorSetpoint = ElevatorConstants.LevelTwoSetpoint;
        break;
      case L3:
        elevatorSetpoint = ElevatorConstants.LevelThreeSetpoint;
        break;
      case L4:
        elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
        break;
      case HP:
        elevatorSetpoint = ElevatorConstants.HPsetpoint;
        break;
      case MANUAL:
        break;
      case OFF:
        elevatorSetpoint = 0;
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    handleLeft();
    // handleRight();
    encoderPosition = encoderLeft.getPosition();
    logValues();
    checkTunableValues();
    switch (isLeftOutOfBounds()) {
      case BADBADBAD:
        // leftElevatorMotor.setVoltage(0);
        // System.out.println("BADDBADBADBAD");
        break;
      default:
        break;
    }

    var ffOutput = ffElevate.calculateWithVelocities(nextVelocity, nextNextVelocity);

    if (ffOutput < ffElevate.calculate(nextVelocity)) {
      ffOutput = ffElevate.calculate(nextVelocity); // the docmentation said "calculateWithVelocities" is inaccurate for
                                                    // values
                                                    // around 0
    }
    var leftpidOutput = pid.calculate(encoderPosition, this.elevatorSetpoint);

    leftElevatorMotor.set(-ffOutput - leftpidOutput);
    rightElevatorMotor.set(ffOutput + leftpidOutput);

    SmartDashboard.putNumber("Elevator velocity", nextVelocity);
    SmartDashboard.putNumber("Elevator PID output left", leftpidOutput);
    SmartDashboard.putNumber("Elevator PID ouput right", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator FF Output", ffOutput);

  }
}