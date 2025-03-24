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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;

public class Elevator extends SubsystemBase {

  public SparkMax leftElevatorMotor = new SparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
  public SparkMax rightElevatorMotor = new SparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  // private PIDController pid = new
  // PIDController(ElevatorConstants.elevatorPID[0],
  // ElevatorConstants.elevatorPID[1],
  // ElevatorConstants.elevatorPID[2]);
  public ProfiledPIDController pid = new ProfiledPIDController(ElevatorConstants.elevatorPID[0],
      ElevatorConstants.elevatorPID[1],
      ElevatorConstants.elevatorPID[2],
      new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAccel));
  public ElevatorFeedforward ffElevate = new ElevatorFeedforward(ElevatorConstants.elevatorSGV[0],
      ElevatorConstants.elevatorSGV[1], ElevatorConstants.elevatorSGV[2], ElevatorConstants.elevatorSGV[3]);

  private final RelativeEncoder encoderRight;
  // private final RelativeEncoder encoderLeft;

  private Timer timer = new Timer();
  public double encoderPosition;
  private double nextVelocity = 0.0;
  private double nextNextVelocity = 0.0;
  public double elevatorSetpoint = 0.5;
  private double positionRateOfChange = 0;
  private double changingSetpoint = 0;

  // private double leftPower = 0;
  // private double rightPower = 0;
  private ElevateMode elevateMode = ElevateMode.OFF;
  private boolean isLeftDone = false;
  // private boolean isRightDone = false;

  /* Tunable Values */
  private LoggedTunableNumber elevatorP = new LoggedTunableNumber("elevatorP",
  ElevatorConstants.elevatorPID[0]);
  private LoggedTunableNumber elevatorI = new LoggedTunableNumber("elevatorI",
  ElevatorConstants.elevatorPID[1]);
  private LoggedTunableNumber elevatorD = new LoggedTunableNumber("elevatorD",
  ElevatorConstants.elevatorPID[2]);
  // private LoggedTunableNumber elevatorS = new LoggedTunableNumber("elevatorS",
  // ElevatorConstants.elevatorSGV[0]);
  // private LoggedTunableNumber elevatorG = new LoggedTunableNumber("elevatorG",
  // ElevatorConstants.elevatorSGV[1]);
  // private LoggedTunableNumber elevatorV = new LoggedTunableNumber("elevatorV",
  // ElevatorConstants.elevatorSGV[2]);
  // private LoggedTunableNumber elevatorA = new LoggedTunableNumber("elevatorA",
  // ElevatorConstants.elevatorSGV[3]);
  // private LoggedTunableNumber elevatorLevel = new LoggedTunableNumber("changing
  // setpoint", elevatorSetpoint);

  public Elevator() {
    setupMotors();
    encoderRight = rightElevatorMotor.getEncoder();
    // encoderLeft = leftElevatorMotor.getEncoder();
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
    if (encoderRight.getPosition() <= ElevatorConstants.min) {
      this.elevatorSetpoint = ElevatorConstants.min;
    } else if (encoderRight.getPosition() >= ElevatorConstants.max) {
      this.elevatorSetpoint = ElevatorConstants.max;
    } else {
      this.elevatorSetpoint = goal;
    }

  }

  public double getSetpoint() {
    return this.elevatorSetpoint;
  }

  public void SET(double value) {
    encoderRight.setPosition(value);
  }

  public void reset() {
    encoderRight.setPosition(0);
  }

  public double ActualPositionLeader() {
    return encoderRight.getPosition();
  }

  public double getDesiredPositionLeader() {
    return positionRateOfChange;
  }

  private void resetEncoders() {
    encoderRight.setPosition(0);
    // encoderRight.setPosition(0);
  }

  public void logValues() {
    SmartDashboard.putNumber("Actual Elevator Position Left", encoderRight.getPosition());
    SmartDashboard.putNumber("Desired Elevator Position", pid.getSetpoint().position);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(encoderRight.getPosition(), encoderRight.getVelocity());
  }

  public void runState(TrapezoidProfile.State state) {
    elevatorSetpoint = state.position;
    this.nextVelocity = state.velocity;
  }

  public void checkTunableValues() {
    // if (Constants.enableTunableValues) {

    if (elevatorP.hasChanged() || elevatorI.hasChanged() ||
    elevatorD.hasChanged()) {
    pid.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());
    }
    // if (elevatorS.hasChanged() || elevatorG.hasChanged() ||
    // elevatorV.hasChanged()) {
    // ffElevate = new ElevatorFeedforward(elevatorS.get(), elevatorG.get(),
    // elevatorV.get());
    // }
    // }
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
    return outOfBounds(encoderRight.getPosition());
  }

  
  public boolean atGoal() {
    return Math.abs(encoderRight.getPosition() - elevatorSetpoint) < ElevatorConstants.elevatorTolerance;
  }

  @Override
  public void periodic() {
    encoderPosition = encoderRight.getPosition();
    checkTunableValues();
    logValues();
    

    // var ffOutput = ffElevate.calculateWithVelocities(nextVelocity,
    // nextNextVelocity);

    // if (ffOutput < ffElevate.calculate(nextVelocity)) {
    // var ffOutput = ffElevate.calculate(); // the docmentation said "calculateWithVelocities" is inaccurate
                                                      // for
    // values around 0
    // }
    // var leftpidOutput = pid.calculate(encoderPosition, this.elevatorSetpoint);

    // leftElevatorMotor.set(-leftpidOutput);
    // rightElevatorMotor.set(leftpidOutput);

    SmartDashboard.putNumber("Elevator velocity", leftElevatorMotor.get());
    // SmartDashboard.putNumber("Elevator PID output left", leftpidOutput);
    SmartDashboard.putNumber("Elevator's Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Current", leftElevatorMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Elevator FF Output", ffOutput);

  }
}