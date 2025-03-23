package frc.robot.subsystems.Drive;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class ChoreoAutos {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    AutoFactory autoFactory = drivetrain.createAutoFactory();

    public AutoRoutine Madtown() {
        AutoRoutine routine = autoFactory.newRoutine("Madtown");

        AutoTrajectory scorePreload = routine.trajectory("MT1");
        AutoTrajectory getFirstTrajectory = routine.trajectory("MT2");

        return routine;
    }
}
