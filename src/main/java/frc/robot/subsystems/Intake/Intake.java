package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants.ScoringMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
    public enum WantedMode {
        INTAKE_ALGAE,
        SCORE_ALGAE,
        INTAKE_CORAL,
        SCORE_CORAL_BATTERYSIDE,
        SCORE_CORAL_PIVOTSIDE,
        SCORE_CORAL_L1,
        IDLE
    }
    private enum SystemMode {
        INTAKING_ALGAE,
        SCORING_ALGAE,
        INTAKING_CORAL,
        SCORING_CORAL_BATTERYSIDE,
        SCORING_CORAL_PIVOTSIDE,
        SCORING_CORAL_L1,
        IDLING
    }


    private static TalonFX intake = new TalonFX(61);

    private static TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    private static DigitalInput beamBreak = new DigitalInput(1);

    private static Timer pulseTimer = new Timer();
    public static boolean modified = false;

    private WantedMode wantedMode = WantedMode.IDLE;
    private SystemMode systemMode = SystemMode.IDLING;

    public Intake() {
        setupMotors();
        pulseTimer.reset();
    }

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public void setupMotors() {
        // Apply Configs

        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intake.setNeutralMode(NeutralModeValue.Brake);
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public void hpIntake(double value) {
        if (beamBreak.get()) {
            // follower.set(value);
            intake.set(value);
        } else {
            // leader.set(value);
            // follower.set(0);
            intake.set(0);
        }

    }

    public void setModify(boolean yesOrNo) {

    }

    // Outtakes through the black wheels
    public void outTake(double value) {
        // follower.set(value);

        intake.set(value);

    }

    public void stop() {
        // leader.set(0);
        // follower.set(0);

        intake.set(0);
    }
    public void setWantedIntakeMode(WantedMode desiredMode) {
        this.wantedMode = desiredMode;
    }

    private SystemMode changeCurrentMode() {
        return switch (wantedMode) {
            case INTAKE_ALGAE:
                if (!hasCoral()){
                    yield SystemMode.INTAKING_ALGAE;
                } else {
                    yield SystemMode.IDLING;
                }
            case SCORE_ALGAE:
                yield SystemMode.SCORING_ALGAE;
            case INTAKE_CORAL:
                if(!hasCoral()) {
                    yield SystemMode.INTAKING_CORAL;
                } else {
                    yield SystemMode.IDLING;
                }
            // case SCORE_CORAL:
            // if(hasCoral()){
            //     if (mContainer.drivetrain.decideScoringMode() == ScoringMode.MODIFIED) {
            //         if (mContainer.operator.x().getAsBoolean()) {
            //             yield SystemMode.SCORING_CORAL_L1;
            //         } else {
            //         yield SystemMode.SCORING_CORAL_BATTERYSIDE;
            //         }
            //     } else {
            //         yield SystemMode.SCORING_CORAL_PIVOTSIDE;
            //     }
            // } else {
            //     yield SystemMode.IDLING;
            // }
            case SCORE_CORAL_BATTERYSIDE:
            if(hasCoral()){
                yield SystemMode.SCORING_CORAL_BATTERYSIDE;
            } else {
                yield SystemMode.IDLING;
            }
            case SCORE_CORAL_PIVOTSIDE:
            if(hasCoral()){
                yield SystemMode.SCORING_CORAL_PIVOTSIDE;
            } else {
                yield SystemMode.IDLING;
            }
            case SCORE_CORAL_L1:
            if(hasCoral()){
                yield SystemMode.SCORING_CORAL_L1;
            } else {
                yield SystemMode.IDLING;
            }
            case IDLE:
                yield SystemMode.IDLING;    
        };
    }

    private void applyState() {
        double motorSpeed = 0.0;
        switch (systemMode) {
            case SCORING_CORAL_L1:
                motorSpeed = .2;
                break;
            case SCORING_CORAL_BATTERYSIDE:
                motorSpeed = -0.7;
                break;
            case SCORING_CORAL_PIVOTSIDE:
                motorSpeed = 0.7;
                break;
            case SCORING_ALGAE:
                intakeConfig.CurrentLimits.SupplyCurrentLimit = 70;
                motorSpeed = -0.7;
                break;
            case INTAKING_CORAL:
                intakeConfig.CurrentLimits.SupplyCurrentLimit = 70;
                if(!hasCoral()) {
                    motorSpeed = -0.4;
                } else {
                    motorSpeed = 0.0;
                }
                break;
            case INTAKING_ALGAE:
                intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
                motorSpeed = 0.7;
                break;
            case IDLING:
                intakeConfig.CurrentLimits.SupplyCurrentLimit = 70;
                motorSpeed = 0.0;
                break;
        }
        SmartDashboard.putNumber("motorspeed", motorSpeed);
        intake.set(motorSpeed);
    }

    @Override
    public void periodic() {
        systemMode = changeCurrentMode();
        applyState();
        SmartDashboard.putBoolean("Beamy", beamBreak.get());
        SmartDashboard.putString("INTAKE WANTED STATE", wantedMode.toString());
        SmartDashboard.putString("INTAKE SYSTEM STATE", systemMode.toString());
    }
}
