package frc.lib.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// import com.revrobotics.spark.CANSparkLowLevel;
// import com.revrobotics.SparkMax;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus0;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus1;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing
   * the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>
   * See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor           The motor to adjust the status frame periods on.
   * @param usage           The status frame feedack to enable. kAll is the
   *                        default when a SparkMax is
   *                        constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setSparkMaxBusUsage(
      SparkMax motor, SparkMaxConfig config, Usage usage, boolean enableFollowing) {
    if (enableFollowing) {
      config.signals.primaryEncoderPositionPeriodMs(10);
    } else {
      config.signals.primaryEncoderPositionPeriodMs(500);
    }

    if (usage == Usage.kAll) {
      //SparkLowLevel.PeriodicStatus0
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(50);

      // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
      // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);

      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 
    
    else if (usage == Usage.kPositionOnly) {
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(500);
      
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
    } 
    
    else if (usage == Usage.kVelocityOnly) {

      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(500);

      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
    } 
    
    else if (usage == Usage.kMinimal) {
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
      // motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);

      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(500);
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing
   * the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>
   * See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a
   *              SparkMax is
   *              constructed.
   */
  public static void setSparkMaxBusUsage(SparkMax motor, SparkMaxConfig config, Usage usage) {
    setSparkMaxBusUsage(motor, config, usage, false);
  }
}
