package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

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

    private static TalonFX intake = new TalonFX(0);
    
    private static MotorOutputConfigs intakeConfig = new MotorOutputConfigs();

    private static DigitalInput beamBreak = new DigitalInput(1);

    private static Timer pulseTimer = new Timer();
    public static boolean modified = false;

    public Intake() {
        setupMotors();
        pulseTimer.reset();
    }

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public void setupMotors() {
        //Apply Configs

        intake.setNeutralMode(NeutralModeValue.Brake);        
        intakeConfig.Inverted = InvertedValue.Clockwise_Positive;
    }

    public void hpIntake(double value) {
        if (beamBreak.get()) {
            //follower.set(value);
            intake.set(value);
        } else {
            // leader.set(value);
            //follower.set(0);
            intake.set(0);
        }

    }

    public void setModify(boolean yesOrNo) {

    }

    // Outtakes through the black wheels
    public void outTake(double value) {
        //follower.set(value);

        intake.set(value);

    }

    public void stop() {
        // leader.set(0);
        // follower.set(0);

        intake.set(0);
    }

    @Override
    public void periodic() {
    }
}
