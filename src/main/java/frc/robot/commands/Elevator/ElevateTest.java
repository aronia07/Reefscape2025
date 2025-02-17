package frc.robot.commands.Elevator;

import java.lang.annotation.ElementType;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevateTest extends Command {
    protected final Elevator elevator_y;
    private State initialState;
    private DoubleSupplier doubleSupplier;
    private TrapezoidProfile profiler_y = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAccel));
    private Timer y_timer = new Timer();
    
    public ElevateTest(Elevator elevator){
        elevator_y = elevator;

        this.addRequirements(elevator_y);
    }

    @Override
    public void initialize() {
        y_timer.reset();
        y_timer.start();
        
        
        // if (level == 4) {
        //     elevator_y.setMode(ElevateMode.L4);
        // } else if (level == 3) {
        //     elevator_y.setMode(ElevateMode.L3);
        // } else if (level == 2) {
        //     elevator_y.setMode(ElevateMode.L2);
        // } else if (level == 1) {
        //     elevator_y.setMode(ElevateMode.L1);
        // } else if (level == 0) {
        //     elevator_y.setMode(ElevateMode.HP);
        // } else if (level == 5) {
        //     elevator_y.setMode(ElevateMode.DOWN);
        // } else if (level == 6) {
        //     elevator_y.setMode(ElevateMode.UP);
        // } else if (level == 7) {
        //     elevator_y.setMode(ElevateMode.TEST);
        // }
        // elevator_y.setMode(ElevateMode.UP);
        // initialState = elevator_y.getCurrentState();
        
    }
    @Override
    public void execute() {
        
        // var nextState = profiler_y.calculate(y_timer.get(), 
        //     initialState, 
        //     new TrapezoidProfile.State(Elevator.elevatorSetpoint, 0));

        // elevator_y.runState(nextState);
        elevator_y.SET(0);
    }

    @Override
    public void end(boolean interrupted){
        
        elevator_y.setMode(ElevateMode.OFF);
    }
    
}
