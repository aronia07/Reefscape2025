package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    public SparkMax climberMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);
    public SparkMaxConfig climberConfig = new SparkMaxConfig();
    public RelativeEncoder climberEncoder;
    public boolean climberCanMove = true;

    public Climber() {
        setupMotors();
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0);
    }

    public void setupMotors() {
        climberConfig.smartCurrentLimit(80, 80)
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void checkPosition() {
        climberEncoder.getPosition();
    }

    public void up(double value) {
        if (climberEncoder.getPosition() < Constants.ClimberConstants.climberMax) {
            climberMotor.set(value);
        } else {
            climberMotor.set(0);
        }
    }

    public void down(double value) {
        if (climberEncoder.getPosition() > Constants.ClimberConstants.climberMin) {
            climberMotor.set(-value);
        } else {
            climberMotor.set(0);
        }
    }

    public void go(double value) {
        climberMotor.set(value);
    }

    @Override
    public void periodic() {
        climberEncoder.getPosition();
        SmartDashboard.putNumber("encder reading", climberEncoder.getPosition());
    }

}
