
//Imports
package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.ArmWantedMode;
import frc.robot.Constants.ArmConstants.SystemMode;
import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;

public class Arm extends SubsystemBase {
  // Initialize motors
  private final SparkMax leader = new SparkMax(Constants.ArmConstants.leaderID, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(Constants.ArmConstants.followerID, MotorType.kBrushless);

  private final SparkMaxConfig armLeaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig armFollowerConfig = new SparkMaxConfig();

  // Initialize sensors
  private final ArmEncoderThroughbore encoder = new ArmEncoderThroughbore(Constants.ArmConstants.encoderID);
  public static Rotation2d encoderPosition = new Rotation2d();
  private DigitalInput beamy = Constants.beamy;

  private ArmWantedMode wantedMode = ArmWantedMode.IDLE;
  private SystemMode systemMode = SystemMode.HIGH_IDLE;

  // PID controller + feedforward initialization
  private final ProfiledPIDController pid = new ProfiledPIDController(Constants.ArmConstants.armPID[0],
      Constants.ArmConstants.armPID[1],
      Constants.ArmConstants.armPID[2],
      new TrapezoidProfile.Constraints(
          Constants.ArmConstants.maxVelocityPerSecond.getRadians(),
          Constants.ArmConstants.maxAcceleration.getRadians()));
  private ArmFeedforward ffModel = new ArmFeedforward(
      Constants.ArmConstants.armSGV[0],
      Constants.ArmConstants.armSGV[1],
      Constants.ArmConstants.armSGV[2]);

  // Motion profiling
  private Rotation2d setpoint = new Rotation2d();
  private Rotation2d velocity = new Rotation2d();
  private Rotation2d goal = new Rotation2d();

  // Tunable values
  // private LoggedTunableNumber armP = new LoggedTunableNumber("armP",
  // Constants.ArmConstants.armPID[0]);
  // private LoggedTunableNumber armI = new LoggedTunableNumber("armI",
  // Constants.ArmConstants.armPID[1]);
  // private LoggedTunableNumber armD = new LoggedTunableNumber("armD",
  // Constants.ArmConstants.armPID[2]);
  // private LoggedTunableNumber armS = new LoggedTunableNumber("armS",
  // Constants.ArmConstants.armSGV[0]);
  // private LoggedTunableNumber armG = new LoggedTunableNumber("armG",
  // Constants.ArmConstants.armSGV[1]);
  // private LoggedTunableNumber armV = new LoggedTunableNumber("armV",
  // Constants.ArmConstants.armSGV[2]);

  public Arm() {
    setupMotors();
    // Set offset + get encoders position
    encoder.setOffset(Constants.ArmConstants.offset);
    encoderPosition = encoder.getAbsolutePosition();
    runSetpoint(getEncoderPosition());
  }

  // Motor Set-Up
  private void setupMotors() {

    armLeaderConfig.voltageCompensation(12).smartCurrentLimit(50, 30).inverted(true).idleMode(IdleMode.kBrake);
    leader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // leader.restoreFactoryDefaults();
    CANSparkMaxUtil.setSparkMaxBusUsage(leader, armLeaderConfig, Usage.kPositionOnly);

    CANSparkMaxUtil.setSparkMaxBusUsage(follower, armFollowerConfig, Usage.kPositionOnly);
    armFollowerConfig.voltageCompensation(12).smartCurrentLimit(50, 30).inverted(false).idleMode(IdleMode.kBrake);
    follower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setIntegratorRange(-0.05, 0.05);
  }

  // Resets the PID's i value
  public void resetI() {
  }

  // returns the encoder's position
  public static Rotation2d getEncoderPosition() {
    return encoderPosition;
  }

  // Checks to see if the PID and SGV values have changed.
  // Updates the value if it's been changed
  // public void checkTunableValues() {
  // // if (!Constants.enableTunableValues)
  // // return;

  // if (armP.hasChanged() || armI.hasChanged() || armD.hasChanged()) {
  // pid.setPID(armP.get(), armI.get(), armD.get());
  // }

  // if (armS.hasChanged() || armG.hasChanged() || armV.hasChanged()) {
  // ffModel = new ArmFeedforward(armS.get(), armG.get(), armV.get());
  // }
  // }

  // Sets the arm's goal
  public void setGoal(Rotation2d goal) {
    this.goal = goal;
  }

  // Check if arm is at goal
  public boolean atGoal() {
    return Math.abs(getEncoderPosition().getRadians() - goal.getRadians()) < Constants.ArmConstants.tolernace
        .getRadians();
  }

  // Ensures the arm remains within its minimum and maximum angle values
  public void runSetpoint(Rotation2d setpoint) {
    if (setpoint.getRadians() < Constants.ArmConstants.min.getRadians()) {
      this.setpoint = Constants.ArmConstants.min;
    } else if (setpoint.getRadians() > Constants.ArmConstants.max.getRadians()) {
      this.setpoint = Constants.ArmConstants.max;
    } else {
      this.setpoint = setpoint;
    }
  }

  // Set arm velocity
  public void runVelocity(Rotation2d velocity) {
    this.velocity = velocity;
  }

  // Update the profile's state
  public void runState(TrapezoidProfile.State state) {
    runSetpoint(Rotation2d.fromRadians(state.position));
    runVelocity(Rotation2d.fromRadians(state.velocity));
  }

  // Get the current trapezoi
  public TrapezoidProfile.State getCurrenState() {
    return new TrapezoidProfile.State(encoderPosition.getRadians(), this.velocity.getRadians());
  }

  // Checks if arm is within tolerance
  public boolean atSetpoint() {
    return Math.abs(getEncoderPosition().getRadians() - setpoint.getRadians()) < Constants.ArmConstants.tolernace
        .getRadians();
  }

  // Get setpoint
  public Rotation2d getSetpoint() {
    return setpoint;
  }

  // Get velocity
  public Rotation2d getVelovity() {
    return velocity;
  }

  public boolean hasCoral() {
    return !beamy.get();
  }

  // Logs values to SmartDashboard/Glass
  private void logValues() {
    SmartDashboard.putNumber("Arm Actual Angle", getEncoderPosition().getDegrees());
    SmartDashboard.putNumber("Arm Desired Angle", setpoint.getDegrees());
    SmartDashboard.putNumber("Arm Desired Speed", velocity.getDegrees());
    SmartDashboard.putNumber("Arm current", leader.getOutputCurrent());
    SmartDashboard.putNumber("arm output", leader.getAppliedOutput());
    SmartDashboard.putString("ARM WANTED MODE", wantedMode.toString());
    SmartDashboard.putString("ARM SYSTEM MODE", systemMode.toString());
  }

  public void setWantedArmMode(ArmWantedMode desiredMode) {
    this.wantedMode = desiredMode;
  }

  private SystemMode changeCurrentSystemMode() {
    return switch (wantedMode) {
      case IDLE:
        if (!hasCoral() || (systemMode == SystemMode.INTAKING_CORAL)) {
          yield SystemMode.LOW_IDLE;
        } else {
          yield SystemMode.HIGH_IDLE;
        }
      case INTAKE_CORAL:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.INTAKING_CORAL;
          }
        }
      case INTAKE_ALGAE:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.INTAKING_ALGAE;
          }
        }
      case L1:
        if (hasCoral()) {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L1;
          }
        } else {
          yield SystemMode.HIGH_IDLE;
        }
      case L2_CORAL:
        if (hasCoral()) {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L2_CORAL;
          }
        } else {
          yield SystemMode.HIGH_IDLE;
        }
      case L2_ALGAE_BATTERY:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L2_ALGAE_BATTERY;
          }
        }
      case L2_ALGAE_PIVOT:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          // if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L2_ALGAE_PIVOT;
          // }
        }
      case L3_CORAL_BATTERY:
        if (hasCoral()) {
          // if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L3_CORAL_BATTERY;
          // }
        } else {
          yield SystemMode.HIGH_IDLE;
        }
      case L3_CORAL_PIVOT:
        if (hasCoral()) {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L3_CORAL_PIVOT;
          }
        } else {
          yield SystemMode.HIGH_IDLE;
        }
      case L3_ALGAE_BATTERY:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L3_ALGAE_BATTERY;
          }
        }
      case L3_ALGAE_PIVOT:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L3_ALGAE_PIVOT;
          } else {
            yield systemMode;
          }
        }
      case L4_CORAL_BATTERY:
        if (hasCoral()) {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L4_CORAL_BATTERY;
          }
        } else {
          yield SystemMode.LOW_IDLE;
        }

      case L4_CORAL_PIVOT:
        if (hasCoral()) {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_L4_CORAL_PIVOT;
          }
        } else {
          yield SystemMode.LOW_IDLE;
        }
      case ALGAE_BARGE:
        if (hasCoral()) {
          yield SystemMode.HIGH_IDLE;
        } else {
          if(systemMode == SystemMode.HIGH_IDLE || systemMode == SystemMode.LOW_IDLE) {
            yield SystemMode.GOING_ALGAE_BARGE;
          }
        }
      case CLIMB:
        yield SystemMode.CLIMBING;
      // case HIGH_IDLE:
      //   yield SystemMode.HIGH_IDLE;
      // case LOW_IDLE:
      //   yield SystemMode.LOW_IDLE;
    };
  }

  private void applyState() {
    switch (systemMode) {
      case INTAKING_CORAL:
        setpoint = new Rotation2d(Units.degreesToRadians(-7));
        break;
      case INTAKING_ALGAE:
        break;
      case GOING_L1:
        setpoint = new Rotation2d(Units.degreesToRadians(20));
        break;
      case GOING_L2_CORAL:
        setpoint = new Rotation2d(Units.degreesToRadians(45));
        break;
      case GOING_L2_ALGAE_PIVOT:
        setpoint = new Rotation2d(Units.degreesToRadians(88.67));
        break;
      case GOING_L2_ALGAE_BATTERY:
        setpoint = new Rotation2d(Units.degreesToRadians(25));
        break;
      case GOING_L3_ALGAE_BATTERY:
        setpoint = new Rotation2d(Units.degreesToRadians(65));
        break;
      case GOING_L3_ALGAE_PIVOT:
        setpoint = new Rotation2d(Units.degreesToRadians(87));
        break;
      case GOING_L3_CORAL_BATTERY:
        setpoint = new Rotation2d(Units.degreesToRadians(55));
        break;
      case GOING_L3_CORAL_PIVOT:
        setpoint = new Rotation2d(Units.degreesToRadians(77));
        break;
      case GOING_L4_CORAL_BATTERY:
        setpoint = new Rotation2d(Units.degreesToRadians(77));
        break;
      case GOING_L4_CORAL_PIVOT:
        setpoint = new Rotation2d(Units.degreesToRadians(82));
        break;
      case GOING_ALGAE_BARGE:
        setpoint = new Rotation2d(Units.degreesToRadians(78));
        break;
      case LOW_IDLE:
        setpoint = new Rotation2d(Units.degreesToRadians(20));
        break;
      case HIGH_IDLE:
        setpoint = new Rotation2d(Units.degreesToRadians(60));
        break;
      case CLIMBING:
        setpoint = new Rotation2d(Units.degreesToRadians(-11));
        break;
    }
  }

  @Override
  public void periodic() {
    systemMode = changeCurrentSystemMode();
    applyState();
    encoderPosition = encoder.getAbsolutePosition(); // Finds the exact position of the encoder
    logValues(); // Logs values to SmartDashboard/Glass
    // checkTunableValues(); //Updates PID and FF values

    var ffOutput = ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // Calculates Feedforward output
    var pidOutput = pid.calculate(getEncoderPosition().getRadians(), setpoint.getRadians()); // calculates PID output

    // SmartDashboard.putNumber("ffoutput arm", ffOutput); //Displays the FF output
    // calculated above on Smartdahsboard/Glass

    // PID+FF output on the leader and follower motors
    leader.set(ffOutput + pidOutput);
    follower.set(ffOutput + pidOutput);
    // follower.set(ffOutput + pidOutput);
  }

}
