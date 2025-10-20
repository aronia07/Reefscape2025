package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.Constants.ElevatorConstants.ElevatorWantedMode;
import frc.robot.Constants.ElevatorConstants.SystemMode;
import frc.robot.subsystems.Intake.Intake;

public class Elevator extends SubsystemBase {

  public SparkMax leftElevatorMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
  public SparkMax rightElevatorMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  public ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.elevatorPID[0],
      ElevatorConstants.elevatorPID[1],
      ElevatorConstants.elevatorPID[2],
      new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAccel));
  public ElevatorFeedforward ffElevate = new ElevatorFeedforward(ElevatorConstants.elevatorSGV[0],
      0, ElevatorConstants.elevatorSGV[2], ElevatorConstants.elevatorSGV[3]);

  private final RelativeEncoder encoderLeft;
  // private final RelativeEncoder encoderLeft;

  public double encoderPosition;
  public double elevatorSetpoint = 1;
  private double positionRateOfChange = 0;

  // private double leftPower = 0;
  // private double rightPower = 0;
  private ElevateMode elevateMode = ElevateMode.OFF;
  private ElevatorWantedMode wantedMode = ElevatorWantedMode.IDLE;
  private SystemMode systemMode = SystemMode.IDLE;
  private DigitalInput beamy = Constants.beamy;
  // private boolean isLeftDone = false;
  // private boolean isRightDone = false;

  /* Tunable Values */
  private LoggedTunableNumber elevatorP = new LoggedTunableNumber("elevatorP",
      ElevatorConstants.elevatorPID[0]);
  private LoggedTunableNumber elevatorI = new LoggedTunableNumber("elevatorI",
      ElevatorConstants.elevatorPID[1]);
  private LoggedTunableNumber elevatorD = new LoggedTunableNumber("elevatorD",
      ElevatorConstants.elevatorPID[2]);
  private LoggedTunableNumber elevatorS = new LoggedTunableNumber("elevatorS",
      ElevatorConstants.elevatorSGV[0]);
  private LoggedTunableNumber elevatorG = new LoggedTunableNumber("elevatorG",
      ElevatorConstants.elevatorSGV[1]);
  private LoggedTunableNumber elevatorV = new LoggedTunableNumber("elevatorV",
      ElevatorConstants.elevatorSGV[2]);
  private LoggedTunableNumber elevatorA = new LoggedTunableNumber("elevatorA",
      ElevatorConstants.elevatorSGV[3]);
  // private LoggedTunableNumber elevatorLevel = new LoggedTunableNumber("changing
  // setpoint", elevatorSetpoint);

  public Elevator() {
    setupMotors();
    encoderLeft = leftElevatorMotor.getEncoder();
    // encoderLeft = leftElevatorMotor.getEncoder();
    resetEncoders();

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
    if (encoderLeft.getPosition() <= ElevatorConstants.min) {
      this.elevatorSetpoint = ElevatorConstants.min;
    } else if (encoderLeft.getPosition() >= ElevatorConstants.max) {
      this.elevatorSetpoint = ElevatorConstants.max;
    } else {
      this.elevatorSetpoint = goal;
    }
  }

  public double getSetpoint() {
    return this.elevatorSetpoint;
  }

  public void SET(double value) {
    encoderLeft.setPosition(value);
  }

  public void reset() {
    encoderLeft.setPosition(0);
  }

  public double ActualPositionLeader() {
    return encoderLeft.getPosition();
  }

  public double getDesiredPositionLeader() {
    return positionRateOfChange;
  }

  private void resetEncoders() {
    encoderLeft.setPosition(0);
    // encoderLeft.setPosition(0);
  }

  public void logValues() {
    SmartDashboard.putNumber("Actual Elevator Position Left", -encoderLeft.getPosition());
    SmartDashboard.putNumber("Desired Elevator Position", pid.getSetpoint().position);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(encoderLeft.getPosition(), encoderLeft.getVelocity());
  }

  // public void runState(TrapezoidProfile.State state) {
  // elevatorSetpoint = state.position;
  // this.nextVelocity = state.velocity;
  // }

  public void checkTunableValues() {
    // if (Constants.enableTunableValues) {

    if (elevatorP.hasChanged() || elevatorI.hasChanged() ||
        elevatorD.hasChanged()) {
      pid.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());
    }
    if (elevatorS.hasChanged() || elevatorG.hasChanged() ||
        elevatorV.hasChanged() || elevatorA.hasChanged()) {
      ffElevate = new ElevatorFeedforward(elevatorS.get(), elevatorG.get(),
          elevatorV.get(), elevatorA.get());
    }
    // }
  }
  

  public boolean atGoal() {
    return Math.abs(encoderLeft.getPosition() - elevatorSetpoint) < ElevatorConstants.elevatorTolerance;
  }

  public void setWantedElevatorMode(ElevatorWantedMode desiredMode) {
    this.wantedMode = desiredMode;
  }

  public boolean hasCoral() {
    return !beamy.get();
  }

  private SystemMode changeCurrentSystemMode() {
    return switch (wantedMode) {
      case IDLE:
        yield SystemMode.IDLE;
      case L1:
        if (hasCoral()) {
          yield SystemMode.GOING_L1;
        } else {
          yield SystemMode.IDLE;
        }
      case INTAKE_CORAL:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.INTAKING_CORAL;
        }
      case INTAKE_ALGAE:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.INTAKING_ALGAE;
        }
      case L2_CORAL:
        if (hasCoral()) {
          yield SystemMode.GOING_L2_CORAL;
        } else {
          yield SystemMode.IDLE;
        }
      case L2_ALGAE_BATTERY:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.GOING_L2_ALGAE_BATTERY;
        }
      case L2_ALGAE_PIVOT:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.GOING_L2_ALGAE_PIVOT;
        }
      case L3_CORAL_BATTERY:
        if (hasCoral()) {
          yield SystemMode.GOING_L3_CORAL_BATTERY;
        } else {
          yield SystemMode.IDLE;
        }
      case L3_CORAL_PIVOT:
        if (hasCoral()) {
          yield SystemMode.GOING_L3_CORAL_PIVOT;
        } else {
          yield SystemMode.IDLE;
        }
      case L3_ALGAE_BATTERY:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.GOING_L3_ALGAE_BATTERY;
        }
      case L3_ALGAE_PIVOT:
        if (hasCoral()) {
          yield SystemMode.IDLE;
        } else {
          yield SystemMode.GOING_L3_ALGAE_PIVOT;
        }
      case L4_CORAL_BATTERY:
        if (hasCoral()) {
          yield SystemMode.GOING_L4_CORAL_BATTERY;
        } else {
          yield SystemMode.IDLE;
        }
      case L4_CORAL_PIVOT:
        if (hasCoral()) {
          yield SystemMode.GOING_L4_CORAL_PIVOT;
        } else {
          yield SystemMode.IDLE;
        }
      case ALGAE_BARGE:
        yield SystemMode.GOING_ALGAE_BARGE;
    };
  }

  private void applyState() {
    switch (systemMode) {
      case INTAKING_CORAL:
        elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
      case INTAKING_ALGAE:
        elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
      case GOING_L1:
        elevatorSetpoint = ElevatorConstants.LevelOneSetpoint;
        break;
      case GOING_L2_CORAL:
        elevatorSetpoint = ElevatorConstants.LevelTwoSetpoint;
        break;
      case GOING_L2_ALGAE_BATTERY:
        elevatorSetpoint = ElevatorConstants.LevelTwoAlgaeSetpoint;
        break;
      case GOING_L2_ALGAE_PIVOT:
        elevatorSetpoint = ElevatorConstants.LevelTwoAlgaeSetpoint;
        break;
      case GOING_L3_ALGAE_BATTERY:
        elevatorSetpoint = ElevatorConstants.LevelThreeSetpointR;
        break;
      case GOING_L3_ALGAE_PIVOT:
        elevatorSetpoint = ElevatorConstants.LevelThreeAR;
        break;
      case GOING_L3_CORAL_BATTERY:
        elevatorSetpoint = ElevatorConstants.LevelThreeSetpointM;
        break;
      case GOING_L3_CORAL_PIVOT:
        elevatorSetpoint = ElevatorConstants.LevelThreeSetpointR;
        break;
      case GOING_L4_CORAL_BATTERY:
        elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
        break;
      case GOING_L4_CORAL_PIVOT:
        elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
        break;
      case GOING_ALGAE_BARGE:
        elevatorSetpoint = ElevatorConstants.LevelFourSetpoint;
        break;
      case IDLE:
        elevatorSetpoint = 1;
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    systemMode = changeCurrentSystemMode();
    applyState();
    encoderPosition = -encoderLeft.getPosition();
    // checkTunableValues();
    logValues();

    // var ffOutput = ffElevate.calculateWithVelocities(nextVelocity,
    // nextNextVelocity);

    // if (ffOutput < ffElevate.calculate(nextVelocity)) {
    // var ffOutput = ffElevate.calculate(); // the docmentation said
    // "calculateWithVelocities" is inaccurate
    // for
    // values around 0
    // }
    var leftpidOutput = pid.calculate(encoderPosition, this.elevatorSetpoint);

    leftElevatorMotor.set(-leftpidOutput);
    rightElevatorMotor.set(leftpidOutput);

    SmartDashboard.putNumber("Elevator velocity", leftElevatorMotor.get());
    // SmartDashboard.putNumber("Elevator PID output left", leftpidOutput);
    SmartDashboard.putNumber("Elevator's Setpoint", elevatorSetpoint);
    // SmartDashboard.putNumber("Elevator Current", leftElevatorMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Elevator FF Output", ffOutput);

  }
}