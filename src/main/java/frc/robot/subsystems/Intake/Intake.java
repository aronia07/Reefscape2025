package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
    private static SparkMax leader = new SparkMax(IntakeConstants.leaderID, MotorType.kBrushless);
    private static SparkMax follower = new SparkMax(IntakeConstants.followerID, MotorType.kBrushless);

    private static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();

    private static DigitalInput beamBreak = new DigitalInput(1);
    // private static SparkClosedLoopController pid =
    // intake.getClosedLoopController();

    private static Timer pulseTimer = new Timer();
    public static boolean modified = false;

    public Intake() {

        setupMotors();
        pulseTimer.reset();

        // leaderConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(kP.get(), kI.get(), kD.get());
        // pid.setReference(0, ControlType.kVelocity);
    }

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public void setupMotors() {
        /* Applying Configs */
        leaderConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20, 20);
        followerConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(80, 80);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void hpIntake(double value) {
        if (beamBreak.get()) {
            follower.set(value);
        } else {
            // leader.set(value);
            follower.set(0);
        }

    }

    public void setModify(boolean yesOrNo) {

    }

    // Outtakes through the black wheels
    public void outTake(double value) {
        follower.set(value);

    }

    public void stop() {
        leader.set(0);
        follower.set(0);
    }

    @Override
    public void periodic() {
    }
}
