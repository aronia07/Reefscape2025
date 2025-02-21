package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
    private static SparkMax intake = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushless); 
    private static SparkMax deflector = new SparkMax(IntakeConstants.deflectorID, MotorType.kBrushless);
    private static SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private static SparkMaxConfig deflectorConfig = new SparkMaxConfig();
    private static SparkClosedLoopController pid = intake.getClosedLoopController();
    private static LoggedTunableNumber kP = new LoggedTunableNumber("Intake P", IntakeConstants.intakePID[0]);
    private static LoggedTunableNumber kI = new LoggedTunableNumber("Intake I", IntakeConstants.intakePID[1]);
    private static LoggedTunableNumber kD = new LoggedTunableNumber("Intake D", IntakeConstants.intakePID[2]);


    public Intake() {
        
        // intakeConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(kP.get(), kI.get(), kD.get());
        // pid.setReference(0, ControlType.kVelocity);   
    }

    public void setupMotors() {
        /* Applying Configs */
        intakeConfig.inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20, 20);
        deflectorConfig.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50, 50);
        // intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /* Function for updating pid values from dashboard */
    // .
    public static void checkTunableValues() {
        if (!Constants.enableTunableValues)
            return;
        if(kP.hasChanged() || kI.hasChanged() || kD.hasChanged()){
            intakeConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
        }
    }

    public void pulse(double time) {
        if (time % 2 == 0) {
            intake.set(0);
        } else {
            intake.set(.5);
        }
    }

    public void setSpeed(double value) {
        intake.set(value);
    }

    public void stop () {
        intake.set(0);
    }

    @Override
    public void periodic() {

    }
}
