package frc.robot.subsystems.Wrist.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristEncoder {
  public Rotation2d getAbsolutePosition();

  public void setOffset(Rotation2d offset);
}