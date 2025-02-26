package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
    private static SparkMax leader = new SparkMax(IntakeConstants.leaderID, MotorType.kBrushless); 
    private static SparkMax follower = new SparkMax(IntakeConstants.followerID, MotorType.kBrushless);
    
    private static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private static SparkMaxConfig followerConfig = new SparkMaxConfig();

    // private static DigitalInput beamBreak = new DigitalInput(0);
   // private static SparkClosedLoopController pid = intake.getClosedLoopController();

    private static Timer pulseTimer = new Timer();


    public Intake() {
        
        setupMotors();
        pulseTimer.reset();

        // leaderConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(kP.get(), kI.get(), kD.get());
        // pid.setReference(0, ControlType.kVelocity);   
    }

    public void setupMotors() {
        /* Applying Configs */
        leaderConfig.inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20, 20);
        followerConfig.inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50, 50);
        
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    // /* Function for updating pid values from dashboard */
    // // .
    // public static void checkTunableValues() {
    //     if (!Constants.enableTunableValues)
    //         return;
    //     if(kP.hasChanged() || kI.hasChanged() || kD.hasChanged()){
    //         leaderConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    //     }
    // }

    public void pulse(Timer timer, double pulseWidth) {
        timer.reset();

        // if(beamBreak.get()){
        //     while(timer.get() <= pulseWidth){
        //         deflector.set(0.5);
        //     }
        // }
    }


    public void setSpeed(double value) {
        leader.set(value);
        follower.set(value);
    }

    // public void setActualSpeed(double value) {
    //     deflector.set(value);
    // }

    public void stop () {
        leader.set(0);
        follower.set(0);
    }

    // public void stopIntake () {
    //     deflector.set(0);
    // }

    @Override
    public void periodic() {

    }
}
