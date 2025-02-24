package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  // Motors
  private SparkMax wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);
  private SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
  private SparkClosedLoopController wristSparkPID = wristMotor.getClosedLoopController();
  

  private final ArmEncoderThroughbore absoluteEncoder = new ArmEncoderThroughbore(
      Constants.WristConstants.absoluteEncoderPort);
  private double previousAbsoluteEncoder = 0.5;

  // PID
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(WristConstants.maxVelocity,
      WristConstants.maxAccel);
      
  private final static PIDController wristRotationPID = new PIDController(WristConstants.wristPID[0],
      WristConstants.wristPID[1],
      WristConstants.wristPID[2]);

  private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(WristConstants.wristFF[0],
        WristConstants.wristFF[1],
        WristConstants.wristFF[2]);
  
    private Rotation2d wristSetPoint = new Rotation2d();
    public boolean isCorrecting = false;
  
    private static LoggedTunableNumber kP = new LoggedTunableNumber("wristP", WristConstants.wristPID[0]);
    private static LoggedTunableNumber kI = new LoggedTunableNumber("wristI", WristConstants.wristPID[1]);
    private static LoggedTunableNumber kD = new LoggedTunableNumber("wristD", WristConstants.wristPID[2]);
  
    private static LoggedTunableNumber kS = new LoggedTunableNumber("wristK", WristConstants.wristFF[0]);
    private static LoggedTunableNumber kV = new LoggedTunableNumber("wristP", WristConstants.wristFF[1]);
    private static LoggedTunableNumber kA = new LoggedTunableNumber("wristA", WristConstants.wristFF[2]);
  
    public Wrist() {
  
      wristMotorConfig.inverted(true)
          .smartCurrentLimit(30, 30)
          .voltageCompensation(50)
          .idleMode(IdleMode.kBrake);
  
      wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      // Encoder
      // this.wristEncoder = this.wristMotor.getEncoder();
      this.absoluteEncoder.setOffset(WristConstants.wristOffset);
    }
  
    public void checkTunableValues() {
      if (!Constants.enableTunableValues)
        return;
      
      if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
        wristRotationPID.setPID(kP.get(), kI.get(), kD.get());
      }
  
      if (kS.hasChanged() || kV.hasChanged() || kA.hasChanged()) {
        wristFF = new SimpleMotorFeedforward(kS.get(),kV.get(),kA.get());
    }

  }

  public Rotation2d getEncoderPosition() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public void setSetpoint(Rotation2d goal) {
    this.wristSetPoint = goal;
  }

  public void sendValues() {
    SmartDashboard.putNumber("Desired Wrist Position", wristSetPoint.getDegrees());
    SmartDashboard.putNumber("Actual Wrist Position", absoluteEncoder.getAbsolutePosition().getDegrees());
  }

  @Override
  public void periodic() {
    checkTunableValues();
    sendValues();
    
    absoluteEncoder.getAbsolutePosition();
    wristSparkPID.setReference(wristSetPoint.getDegrees(), ControlType.kPosition);

    var ffOutput = wristFF.calculate(wristSetPoint.getRadians(), 3); //@TODO - add a velocity
    var pidOutput = wristRotationPID.calculate(getEncoderPosition().getRadians(), wristSetPoint.getRadians());

    wristMotor.set(ffOutput + pidOutput);


    SmartDashboard.putNumber("Wrist PID Output", pidOutput);
    SmartDashboard.putNumber("Wrist FF Output", ffOutput);
    SmartDashboard.putNumber("Wrist Total Output", ffOutput + pidOutput);
    

  }
}