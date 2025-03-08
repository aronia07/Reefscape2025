package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    public SparkMax climberMotor = new SparkMax(ClimberConstants.climberMotorID, null);
    public SparkMaxConfig climberConfig = new SparkMaxConfig();
    public RelativeEncoder climberEncoder;

    public Climber() {
        setupMotors();
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0);
    }
    public void setupMotors() {
        climberConfig.smartCurrentLimit(40, 40)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12);
        
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void checkPosition() {
        double climberPos = climberEncoder.getPosition();
        if(climberPos > ClimberConstants.climberMax) {
            climberMotor.set(0);
        } else if (climberPos < ClimberConstants.climberMax) {
            climberMotor.set(0);
        }
    }
    public void go(double value) {
        climberMotor.set(value);
    }

    @Override
    public void periodic() {
        checkPosition();
    }

}
