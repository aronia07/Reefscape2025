// package frc.robot.commands.Elevator;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ElevatorConstants.ElevateMode;
// import frc.robot.subsystems.Elevator.Elevator;

// public class ElevateManual extends Command {
//     protected final Elevator elevator_y;
//     private DoubleSupplier doubleSupplier;
    
//     public ElevateManual(DoubleSupplier doubleSupplier, Elevator elevator){
//         elevator_y = elevator;
//         this.doubleSupplier = doubleSupplier;
//         addRequirements(elevator);
//     }


//     @Override
//     public void initialize() {
//       elevator_y.setMode(ElevateMode.MANUAL);
//     }

//     @Override
//     public void execute() {
//         // value of joystick
//         double elevatorStick = MathUtil.applyDeadband(this.doubleSupplier.getAsDouble(), 0.1); 
//         // sensitivity of joystick
//         double elevatorChange = (50/50)*elevatorStick;

//         this.elevator_y.setSetpoint(elevatorChange + this.elevator_y.getSetpoint());
//     }

//     // @Override
//     // public void end(boolean interrupted){
//     //     elevator_y.setMode(ElevateMode.OFF);
//     // }
    
// }
