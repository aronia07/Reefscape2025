package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Encoders.ModuleEncoder;
import frc.robot.subsystems.Drive.Encoders.ModuleEncoderThrifty;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;


public class SwerveModule {
  /* Module details */
  public int moduleNumber;

  /* Motors */
  private TalonFX angleMotor;
  private TalonFX driveMotor;


  /* Encoders and their values */
  private ModuleEncoderThrifty driveEncoder;
  private ModuleEncoderThrifty angleEncoder;
  //private RelativeEncoder integratedAngleEncoder;
  private double lastAngle;

  // /* Controllers */
  public final double[] anglePID;
  public final double[] driveSVA;
  public final double[] drivePID;
  public final PIDController drivePIDController;
  public SimpleMotorFeedforward feedforward;

  private VelocityVoltage drVelocityVoltage;
  private DutyCycleOut drDutyCycleOut;
  private PositionVoltage angPositionVoltage;


  public LoggedTunableNumber driveSpeed;
  
  // For logging
  private double driveSetpoint = 0f;
  private double angleSetpoint = 0f;

  private final TalonFXConfiguration angleConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  private Rotation2d angleOffset;


  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;

    this.anglePID = moduleConstants.anglePID;
    this.driveSVA = moduleConstants.driveSVA;
    this.drivePID = moduleConstants.drivePID;
    this.angleOffset = moduleConstants.angleOffset;
    this.drivePIDController = new PIDController(drivePID[0], drivePID[1], drivePID[2]);
    this.feedforward = new SimpleMotorFeedforward(driveSVA[0], driveSVA[1], driveSVA[2]);
    /* Angle Encoder Config */
    angleEncoder = new ModuleEncoderThrifty(moduleConstants.cancoderID);
    angleEncoder.setOffset(moduleConstants.angleOffset);

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.angleMotorID);
    //integratedAngleEncoder = angleMotor.;
    angleMotor.getConfigurator().apply(angleConfig, 0.050);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    driveMotor.getConfigurator().apply(driveConfig, 0.050);
    configDriveMotor();

    lastAngle = getState().angle.getRotations();

    drVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    angPositionVoltage = new PositionVoltage(0).withSlot(0);
  }


  public void setDesiredState(SwerveModuleState desiredState) {
    
    desiredState = OnboardModuleState.optimize(
        desiredState,
        getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not
    this.setSpeed(desiredState,false);
    this.setAngle(desiredState);
    
    //angleMotor.setControl(angPositionVoltage.withPosition(desiredState.angle.getRotations()));
  }

  public void resetToAbsolute() {
    // double integratedAngleEncoderPosition = angleMotor.getPosition().getValueAsDouble() % 360;
    // double absolutePosition = integratedAngleEncoderPosition - integratedAngleEncoderPosition % 360
    //     + angleEncoder.getAbsolutePosition().getDegrees();
    // angleMotor.setPosition(absolutePosition);  

    double absolutePosition = getAngle().getRotations() - angleOffset.getRotations();
    angleMotor.setPosition(absolutePosition);
  }

  private void configAngleMotor() {
    // Angle motor configuration
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleConfig.Slot0.kP = anglePID[0]; //0.075
    angleConfig.Slot0.kI = anglePID[1];
    angleConfig.Slot0.kD = anglePID[2];

    angleConfig.CurrentLimits.SupplyCurrentLimit = 30;
    angleConfig.Voltage.PeakForwardVoltage = 12.0;
    angleConfig.Voltage.PeakReverseVoltage = -12.0;
    angleMotor.setInverted(true);
    angleMotor.setNeutralMode(NeutralModeValue.Brake);
    
    //angleConfig.Audio.withBeepOnBoot(false);
    
    angleMotor.getConfigurator().apply(angleConfig);
  //resetToAbsolute();
}


private void configDriveMotor() {
    // Drive motor configuration
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    //@TOOD - Get the values required to 
    driveConfig.Slot0.kP = drivePID[0]; //0.2
    driveConfig.Slot0.kI = drivePID[1];
    driveConfig.Slot0.kD = drivePID[2]; 

    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;

    driveMotor.setPosition(0.0);

    // Drive motor configuration.
    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.StatorCurrentLimit = 30; 
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

     // Reset drive encoder
    if(moduleNumber == 1 || moduleNumber == 3){
      driveMotor.setInverted(true);
    }
    else{
      driveMotor.setInverted(false);
    }

    driveMotor.getConfigurator().apply(driveConfig);
}

private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
  if(isOpenLoop){
      drDutyCycleOut.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.setControl(drDutyCycleOut);
  }
  else {
    //driveMotor.set(desiredState.speedMetersPerSecond);
      // drVelocityVoltage.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
      // drVelocityVoltage.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
      //driveMotor.setControl(drVelocityVoltage);
  }
}

  private void setAngle(SwerveModuleState desiredState) {
   //Prevent rotating module if speed is less then 1%. Prevents jittering.
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle.getRotations();   
    
        angleSetpoint = angle;

        //Controlling the swerve pod: testing if conversion factor math needs to be done
        angleMotor.setControl(angPositionVoltage.withPosition(angleSetpoint/Constants.SwerveConstants.angleConversionFactor));

        
      }

  public void logValues() {
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Desired Speed", driveSetpoint);
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Actual Speed", Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference));

    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Angle (From motor in deg)", getAngleCTRE());
    
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Desired Angle", angleSetpoint);
    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " Actual Angle", getAngle().getRotations());

    SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " AngleEncoder Reading:", angleEncoder.getAbsolutePosition().getRotations());
  }

  // public void goToHome() {
  //   Rotation2d angle = getAngle();

  //   angleMotor.setControl(angPositionVoltage.withPosition(angle.getDegrees()-angle.getDegrees()%360));
  //   lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
  // }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getRotations());
  }

public double getAngleCTRE(){
  return 360 * ( (angleMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.angleConversionFactor) % 1);
}
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getSpeed(), this.getAngle());
  }

  public double getSpeed() {
    //In m/s
    return Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference);
  }

  public double getDistance() {
    //@TODO: CHnage
    return (this.driveMotor.getPosition().getValueAsDouble() / 1024) * Constants.SwerveConstants.wheelCircumference;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDistance(), this.getAngle());
  }

  public SwerveModulePosition getRedPosition() {
    return new SwerveModulePosition(this.getDistance(), Rotation2d.fromDegrees(-this.getAngle().getDegrees()));
  }

  public TalonFX getDriveMotor() {
    return this.driveMotor;
  }

  
  public void periodic() {
    //drive pid output
   /* */ var pidOutput = MathUtil.clamp(drivePIDController.calculate(getSpeed(),
        driveSetpoint),
        -Constants.SwerveConstants.maxSpeed,
        Constants.SwerveConstants.maxSpeed);


var ffOutput = feedforward.calculate(driveSetpoint);


// driveController.setReference((pidOutput / Constants.SwerveConstants.maxSpeed)
//     * 12 + ffOutput,
//     ControlType.kVoltage);

    //driveMotor.setControl(drVelocityVoltage);
    //(pidOutput / Constants.SwerveConstants.maxSpeed) * 12 + ffOutput)

SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " PID output", pidOutput);
SmartDashboard.putNumber(Constants.SwerveConstants.moduleNames[moduleNumber] + " FF output", ffOutput);

  





     
  }
}