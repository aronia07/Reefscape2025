// package frc.robot.commands.Wrist;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Wrist.Wrist;

// public class WristMove extends Command {
//     protected Wrist wrist_y = new Wrist();
//     private DoubleSupplier wristSupplier;

//     public WristMove(DoubleSupplier doubleSupplier, Wrist wrist) {
//         this.wrist_y = wrist;
//         this.wristSupplier = doubleSupplier;
//         addRequirements(wrist);

//     }    

//     @Override
//     public void initialize() {
 
//     }

//     @Override
//     public void execute() {
//         double wristY = MathUtil.applyDeadband(wristSupplier.getAsDouble(), 0.1);

//         double wristChange = wristY;

//         this.wrist_y.setSetpoint(wristChange);
//     }
    
// }
