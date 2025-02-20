package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

    // Motors
    private final SparkMax wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);
    private final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    // Encoder
    private final RelativeEncoder wristEncoder;
    private final ArmEncoderThroughbore absoluteEncoder = new ArmEncoderThroughbore(
        Constants.WristConstants.absoluteEncoderPort);
    private double previousAbsoluteEncoder = 0.5;

    //PID
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(WristConstants.maxVelocity, 
                                                                                                WristConstants.maxAccel);
    private final PIDController wristRotationPID = new PIDController(WristConstants.wristPID[0],
                                                        WristConstants.wristPID[1],
                                                        WristConstants.wristPID[2]);
    private final ArmFeedforward wristFF = new ArmFeedforward(WristConstants.wristFF[0], 
                                                            WristConstants.wristFF[1], 
                                                            WristConstants.wristFF[2]);
    // private final PIDConstants intakePIDConstants = Constants.WristConstants.intakePIDConstants;
    // private final PIDController intakePIDController = Constants.WristConstants.intakePIDConstants.getController();
    private double wristSetPoint = 0;
    private double intakeSetVelocity = 0;
    public boolean isCorrecting = false;

    public Wrist() {
    
        // this.intakeMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
        // this.intakeMotor.enableVoltageCompensation(true);
        wristMotorConfig.inverted(true)
                        .smartCurrentLimit(30, 30)
                        .voltageCompensation(50)
                        .idleMode(IdleMode.kBrake);
    
        // Encoder
        this.wristEncoder = this.wristMotor.getEncoder();
        this.absoluteEncoder.setOffset(WristConstants.wristOffset);
        // this.wristEncoder.setConversionFactor(360 / Constants.WristConstants.wristGearRatio);
        // TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    
        // intakeMotor.configFactoryDefault();
        // intakeMotor.configAllSettings(intakeMotorConfiguration);
    
        SmartDashboard.putString("wrist limit", "none");
      }
    
    //   @Override
    //   protected double getMeasurement() {
    //     return this.getAbsoluteEncoder();
    //   }
      public void setSetpoint(double goal) {
        this.wristSetPoint = goal;
      }
    
      @Override
      public void periodic() {
        double ffOutput = wristFF.calculate(0, 0);
        double pidoutput = wristRotationPID.calculate(0, 0);
        this.wristMotor.set(ffOutput + pidoutput);
    
        SmartDashboard.putNumber("Wrist PID Output", pidoutput);
        SmartDashboard.putNumber("Wrist FF Output", ffOutput);
        SmartDashboard.putNumber("Wrist Total Output", ffOutput + pidoutput);
    
      }
    }