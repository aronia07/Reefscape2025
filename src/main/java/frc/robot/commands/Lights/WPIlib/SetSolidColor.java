// package frc.robot.commands.Lights.WPIlib;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
// import edu.wpi.first.wpilibj.util.Color;

// public class SetSolidColor extends Command {
// private final LEDSubsystem_WPIlib ledSubsystem;
// private final Color color;

// public SetSolidColor(LEDSubsystem_WPIlib subsystem, Color color) {
// this.ledSubsystem = subsystem;
// this.color = color;
// addRequirements(subsystem);
// }

// @Override
// public void initialize() {
// ledSubsystem.LED_SolidColor(color);
// }
// @Override
// public void execute(){
// ledSubsystem.LED_SolidColor(color);
// }

// @Override
// public boolean isFinished() {
// return true;
// }
// }