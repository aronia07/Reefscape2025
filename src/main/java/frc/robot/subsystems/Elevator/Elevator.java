package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;

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
  private final RelativeEncoder encoderRight;

  private Timer timer = new Timer();
  public double encoderPosition;
  private double nextVelocity = 0.0;
  private double nextNextVelocity = 0.0;
  public double elevatorSetpoint = 0.05;
  private double positionRateOfChange = 0;

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

  public Elevator() {
    setupMotors();
    encoderLeft = leftElevatorMotor.getEncoder();
    resetEncoders();
    encoderRight = leftElevatorMotor.getEncoder();

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
        .smartCurrentLimit(40, 20)
        .voltageCompensation(12);

    rightConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 20)
        .voltageCompensation(12);
    // rightConfig.apply(leftConfig)
    // .follow(leftElevatorMotor, true);

    leftElevatorMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightElevatorMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // rightElevatorMotor.configure(rightConfig,
    // com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

  }

  public void setMode(ElevateMode mode) {
    elevateMode = mode;
  }

  public void setSetpoint(double goal) {
    elevatorSetpoint = goal;
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
    encoderRight.setPosition(0);
  }

  public void logValues() {
    SmartDashboard.putNumber("Actual Elevator Position Left", encoderLeft.getPosition());
    SmartDashboard.putNumber("Desired Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Actual Elevator Position Right", encoderRight.getPosition());
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(encoderPosition, nextVelocity);
  }

  public void runState(TrapezoidProfile.State state, TrapezoidProfile.State nextState) {
    elevatorSetpoint = state.position;
    nextVelocity = state.velocity;
    nextNextVelocity = nextState.velocity;
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

  public States isRightOutOfBounds() {
    return outOfBounds(-encoderRight.getPosition());
  }

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
        // switch (isLeftOutOfBounds()) {
        // case BADBADBAD:
        // elevatorSetpoint = 0;
        // break;

        // case OKAYUP:
        // if (leftPower > 0) {
        // setRight(0);
        // }
        // break;

        // case OKAYDOWN:
        // if (rightPower < 0) {
        // setRight(0);
        // }
        // break;
        // default:
        // break;
        // }
        break;
      case TEST:
        elevatorSetpoint = ElevatorConstants.test;
      case OFF:
        elevatorSetpoint = 0;
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
        // switch (isLeftOutOfBounds()) {
        // case BADBADBAD:
        // elevatorSetpoint = 0;
        // break;

        // case OKAYUP:
        // if (leftPower > 0) {
        // setRight(0);
        // }
        // break;

        // case OKAYDOWN:
        // if (rightPower < 0) {
        // setRight(0);
        // }
        // break;
        // default:
        // break;
        // }
        break;
      case OFF:
        elevatorSetpoint = 0;
        break;
      default:
        break;
    }
  }
  // private void handleRight() {
  // switch (climberMode) {
  // case HOMING:
  // if (rightClimberMotor.getOutputCurrent() >=
  // ClimberConstants.homingCurrentThreshold) {
  // if (timer.get() < 1)
  // return;
  // stopRight();
  // isRightDone = true;
  // encoderRight.setPosition(0);
  // } else {
  // setRight(ClimberConstants.selfHomeSpeedVoltage);
  // }
  // break;
  // case DEPLOY:
  // if (encoderRight.getPosition() >= ClimberConstants.max) {
  // stopRight();
  // isRightDone = true;
  // } else {
  // setRight(-ClimberConstants.deploySpeed);
  // }
  // break;
  // case RETRACT:
  // if (encoderRight.getPosition() <= ClimberConstants.min) {
  // stopRight();
  // isRightDone = true;
  // } else {
  // setRight(ClimberConstants.retractSpeed);
  // }
  // break;
  // case MANUAL:
  // switch (isRightOutOfBounds()) {
  // case BADBADBAD:
  // stopRight();
  // break;

  // case OKAYUP:
  // if (rightPower > 0) {
  // setRight(0);
  // }
  // break;

  // case OKAYDOWN:
  // if (rightPower < 0) {
  // setRight(0);
  // }
  // break;
  // default:
  // break;
  // }
  // break;
  // case IDLE:
  // stop();
  // break;

  // default:
  // break;
  // }
  // }

  @Override
  public void periodic() {
    handleLeft();
    encoderPosition = encoderLeft.getPosition();
    logValues();
    checkTunableValues();
    switch (isLeftOutOfBounds()) {
      case BADBADBAD:
        leftElevatorMotor.setVoltage(0);
        System.out.println("BADDBADBADBAD");
        return;
      default:
        break;
    }

    var ffOutput = ffElevate.calculateWithVelocities(nextVelocity, nextNextVelocity);
    var rightffOuput = ffElevate.calculateWithVelocities(nextVelocity, nextNextVelocity);

    if (ffOutput < ffElevate.calculate(nextVelocity)) {
      ffOutput = ffElevate.calculate(nextVelocity); // the docmentation said "calculateWithVelocities" is inaccurate
      rightffOuput = ffElevate.calculate(nextVelocity); // with velocities close to 0
    }
    var leftpidOutput = pid.calculate(encoderPosition, elevatorSetpoint);
    var rightpidOutput = pid.calculate(encoderPosition, elevatorSetpoint);

    SmartDashboard.putNumber("Elevator velocity", nextVelocity);
    SmartDashboard.putNumber("Elevator PID output left", leftpidOutput);
    SmartDashboard.putNumber("Elevator PID ouput right", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator FF Output", ffOutput);

    leftElevatorMotor.set(ffOutput + leftpidOutput);
    rightElevatorMotor.set(rightffOuput + rightffOuput);
    // pid.setReference(elevatorSetpoint, ControlType.kPosition,
    // ClosedLoopSlot.kSlot0, ffOutput);

  }
}