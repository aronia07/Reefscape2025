// package frc.robot.subsystems.Wrist;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.servohub.ServoHub.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import frc.robot.subsystems.Arm.Encoders.ArmEncoderThroughbore;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.util.LoggedTunableNumber;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.WristConstants;

// public class Wrist extends SubsystemBase {

//   // Motors
//   private SparkMax wristMotor = new SparkMax(Constants.WristConstants.wristMotorID, MotorType.kBrushless);
//   private SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
//   private SparkClosedLoopController wristSparkPID = wristMotor.getClosedLoopController();
//   // Encoder
//   // private final RelativeEncoder wristEncoder;
//   private final ArmEncoderThroughbore absoluteEncoder = new ArmEncoderThroughbore(
//       Constants.WristConstants.absoluteEncoderPort);
//   private double previousAbsoluteEncoder = 0.5;

//   // PID
//   private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(WristConstants.maxVelocity,
//       WristConstants.maxAccel);
//   private final static PIDController wristRotationPID = new PIDController(WristConstants.wristPID[0],
//       WristConstants.wristPID[1],
//       WristConstants.wristPID[2]);
//   private final ArmFeedforward wristFF = new ArmFeedforward(WristConstants.wristFF[0],
//       WristConstants.wristFF[1],
//       WristConstants.wristFF[2]);

//   private double wristSetPoint = 0;
//   public boolean isCorrecting = false;

//   private static LoggedTunableNumber kP = new LoggedTunableNumber("wristP", WristConstants.wristPID[0]);
//   private static LoggedTunableNumber kI = new LoggedTunableNumber("wristI", WristConstants.wristPID[1]);
//   private static LoggedTunableNumber kD = new LoggedTunableNumber("wristD", WristConstants.wristPID[2]);

//   public Wrist() {

//     wristMotorConfig.inverted(true)
//         .smartCurrentLimit(30, 30)
//         .voltageCompensation(50)
//         .idleMode(IdleMode.kBrake);
//     wristMotorConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());

//     wristMotor.configure(wristMotorConfig, 
//       com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
//       // Encoder
//     // this.wristEncoder = this.wristMotor.getEncoder();
//     this.absoluteEncoder.setOffset(WristConstants.wristOffset);
    
//     SmartDashboard.putString("wrist limit", "none");
//   }

//   public void checkTunableValues() {
//     if (!Constants.enableTunableValues)
//       return;
//     // if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
//     //   wristRotationPID.setPID(kP.get(), kI.get(), kD.get());
//     // }
//     if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
//       wristMotorConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
//     }
//   }

//   public void setSetpoint(double goal) {
//     this.wristSetPoint = goal;
//   }

//   public void sendValues() {
//     SmartDashboard.putNumber("Desired Wrist Position", wristSetPoint);
//     SmartDashboard.putNumber("Actual Wrist Position", absoluteEncoder.getAbsolutePosition().getDegrees());
//   }

//   @Override
//   public void periodic() {
//     checkTunableValues();
//     sendValues();
//     // var ffOutput = wristFF.calculate(0, 0);
//     // var pidoutput = wristRotationPID.calculate(0, 0);
    
//     // this.wristMotor.set(ffOutput + pidoutput);
//     wristSparkPID.setReference(wristSetPoint, ControlType.kPosition);

    
//     // SmartDashboard.putNumber("Wrist PID Output", pidoutput);
//     // SmartDashboard.putNumber("Wrist FF Output", ffOutput);
//     // SmartDashboard.putNumber("Wrist Total Output", ffOutput + pidoutput);

//   }
// }