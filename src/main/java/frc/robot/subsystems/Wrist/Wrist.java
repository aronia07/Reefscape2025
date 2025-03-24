
//Imports
package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.lang.constant.ConstantDesc;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

//import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
//import frc.lib.util.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Wrist.Encoders.WristEncoder;
import frc.robot.subsystems.Wrist.Encoders.WristEncoderThroughbore;

public class Wrist extends SubsystemBase {
  //Initialize motors
  private final SparkMax wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);

  private final SparkMaxConfig wristConfig = new SparkMaxConfig();

  //Initialize encoders
  private final WristEncoderThroughbore encoder = new WristEncoderThroughbore(Constants.WristConstants.absoluteEncoderPort);
  public Rotation2d encoderPosition = new Rotation2d();

  //PID controller + feedforward initialization
  private final PIDController pid = new PIDController(
      Constants.WristConstants.wristPID[0],
      Constants.WristConstants.wristPID[1],
      Constants.WristConstants.wristPID[2]);
  private ArmFeedforward ffModel = new ArmFeedforward(
        Constants.WristConstants.wristFF[0], 
        Constants.WristConstants.wristFF[1], 
        Constants.WristConstants.wristFF[2]);
  
    //Motion profiling 
    private Rotation2d setpoint = new Rotation2d();
    private Rotation2d velocity = new Rotation2d();
    private Rotation2d goal = new Rotation2d();
  
    // Tunable values 
    private LoggedTunableNumber wristP = new LoggedTunableNumber("wristP", Constants.WristConstants.wristPID[0]);
    private LoggedTunableNumber wristI = new LoggedTunableNumber("wristI", Constants.WristConstants.wristPID[1]);
    private LoggedTunableNumber wristD = new LoggedTunableNumber("wristD", Constants.WristConstants.wristPID[2]);
    private LoggedTunableNumber wristS = new LoggedTunableNumber("wristS", Constants.WristConstants.wristFF[0]);
    private LoggedTunableNumber wristG = new LoggedTunableNumber("wristG", Constants.WristConstants.wristFF[1]);
    private LoggedTunableNumber wristV = new LoggedTunableNumber("wristV", Constants.WristConstants.wristFF[2]);
  
  
    public Wrist() {
      setupMotors();
      //Set offset + get encoders position
      encoder.setOffset(Constants.WristConstants.wristOffset);
      encoderPosition = encoder.getAbsolutePosition();
      runSetpoint(getEncoderPosition());
    }
  
  
    //Motor Set-Up
    private void setupMotors() {
  
      wristConfig.voltageCompensation(12).smartCurrentLimit(60, 30).inverted(true).idleMode(IdleMode.kBrake);
      wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //leader.restoreFactoryDefaults();
      CANSparkMaxUtil.setSparkMaxBusUsage(wristMotor, wristConfig, Usage.kPositionOnly);
  
      pid.setIntegratorRange(-0.05, 0.05);
    }
  
    //Resets the PID's i value 
    public void resetI() {
      pid.reset();
    }
  
    //returns the encoder's position
    public Rotation2d getEncoderPosition() {
      return encoderPosition;
    }
  
  
    //Checks to see if the PID and SGV values have changed. 
    //Updates the value if it's been changed
    public void checkTunableValues() {
      // if (!Constants.enableTunableValues)
      //   return;
  
      if (wristP.hasChanged() || wristI.hasChanged() || wristD.hasChanged()) {
        pid.setPID(wristP.get(), wristI.get(), wristD.get());
      }
  
      if (wristS.hasChanged() || wristG.hasChanged() || wristV.hasChanged()) {
        ffModel = new ArmFeedforward(wristS.get(), wristG.get(), wristV.get());
    }
  }

  //Sets the goal
  public void setGoal(Rotation2d goal) {
    this.goal = goal;
  }
//Check if is at goal
  public boolean atGoal() {
    return Math.abs(getEncoderPosition().getRadians() - goal.getRadians()) < Constants.WristConstants.tolernace
        .getRadians();
  }

  //Ensures the wrist remains within its minimum and maximum angle values
  public void runSetpoint(Rotation2d setpoint) {
    if (setpoint.getRadians() < Constants.WristConstants.wristMin.getRadians()) {
      this.setpoint = Constants.WristConstants.wristMin;
    } else if (setpoint.getRadians() > Constants.WristConstants.wristMax.getRadians()) {
      this.setpoint = Constants.WristConstants.wristMax;
    } else {
      this.setpoint = setpoint;
    }
  }

//Set wrist velocity
  public void runVelocity(Rotation2d velocity) {
    this.velocity = velocity;
  }

  //Update the profile's state
  public void runState(TrapezoidProfile.State state) {
    runSetpoint(Rotation2d.fromRadians(state.position));
    runVelocity(Rotation2d.fromRadians(state.velocity));
  }

  //Get the current trapezoi
  public TrapezoidProfile.State getCurrenState() {
    return new TrapezoidProfile.State(encoderPosition.getRadians(), this.velocity.getRadians());
  }

  //Checks if wrist m is within tolerance
  public boolean atSetpoint() {
    return Math.abs(getEncoderPosition().getRadians() - setpoint.getRadians()) < Constants.WristConstants.tolernace
        .getRadians();
  }

  //Get setpoint
  public Rotation2d getSetpoint() {
    return setpoint;
  }

  //Get velocity
  public Rotation2d getVelovity() {
    return velocity;
  }

  //Logs values to SmartDashboard/Glass
  private void logValues() {
    SmartDashboard.putNumber("Wrist Actual Angle", getEncoderPosition().getDegrees());
    SmartDashboard.putNumber("Wrist Desired Angle", setpoint.getDegrees());
    SmartDashboard.putNumber("Wrist Desired Speed", velocity.getDegrees());
    SmartDashboard.putNumber("Wrist current", wristMotor.getOutputCurrent());
  }

  @Override
  public void periodic() {

    encoderPosition = encoder.getAbsolutePosition();  //Finds the exact position of the encoder
    logValues();  //Logs values to SmartDashboard/Glass
    checkTunableValues(); //Updates PID and FF values

  //  var ffOutput = ffModel.calculate(setpoint.getRadians(), velocity.getRadians()); //Calculates Feedforward output
    var pidOutput = pid.calculate(getEncoderPosition().getRadians(), setpoint.getRadians());  //calculates PID output

  //SmartDashboard.putNumber("ffoutput arm", ffOutput); //Displays the FF output calculated above on Smartdahsboard/Glass

    //PID+FF output on the leader and follower motors
    // wristMotor.set(pidOutput+ffOutput);
  }

}
