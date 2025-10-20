package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorWantedMode;
import frc.robot.subsystems.Elevator.Elevator;


public class ElevatorCommand extends Command {
    private Elevator elevator;
    private ElevatorWantedMode desiredMode;

    public ElevatorCommand(Elevator m_elevator, ElevatorWantedMode m_desiredMode) {
        this.elevator = m_elevator;
        this.desiredMode = m_desiredMode;
    }

    @Override
    public void initialize() {
        elevator.setWantedElevatorMode(this.desiredMode);
    }
    @Override
    public void end(boolean interrupted) {
    }
}
