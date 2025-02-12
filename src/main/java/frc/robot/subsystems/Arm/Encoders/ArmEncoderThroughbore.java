package frc.robot.subsystems.Arm.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DutyCycle;

public class ArmEncoderThroughbore implements ArmEncoder {
  private DutyCycleEncoder encoder;
  private Rotation2d offset = new Rotation2d();

  public ArmEncoderThroughbore(int id) {
    encoder = new DutyCycleEncoder(id);
  }

  @Override
  public void setOffset(Rotation2d offset2) {
    this.offset = offset2;
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    return Rotation2d.fromDegrees(180 - (encoder.get() * 360 - offset.getDegrees()));
  }
}