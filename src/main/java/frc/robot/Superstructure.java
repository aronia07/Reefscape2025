// package frc.robot;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CurrentSuperState;
// import frc.robot.Constants.WantedSuperState;
// import frc.robot.Constants.ArmConstants.ArmWantedMode;
// import frc.robot.Constants.ElevatorConstants.ElevatorWantedMode;
// import frc.robot.Constants.IntakeConstants.IntakeWantedMode;
// import frc.robot.Constants.VisionConstants.ScoringMode;

//  class Superstructure extends RobotContainer {

//     private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
//     private CurrentSuperState currentSuperState = CurrentSuperState.IDLING;

    
//     public void setWantedSuperState(WantedSuperState superState) {
//         this.wantedSuperState = superState;
//     }

//     public Command setStateCommand(WantedSuperState superState) {
//         return setStateCommand(superState, false);
//     }

//     public Command setStateCommand(WantedSuperState superState, boolean runIfClimberDeployed) {
//         Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
//         if (!runIfClimberDeployed) {
//             commandToReturn = commandToReturn.onlyIf(() -> currentSuperState != CurrentSuperState.CLIMBING);
//         }
//         return commandToReturn;
//     }
    
    
//     public Command configureButtonBinding(
//             WantedSuperState hasCoralConditionBatterySide,
//             WantedSuperState hasCoralConditionPivotSide,
//             WantedSuperState hasAlgaeConditionBatterySide,
//             WantedSuperState hasAlgaeConditionPivotSide,
//             WantedSuperState noPieceCondition) {
//         return Commands.either(
//                 Commands.either(
//                         Commands.either(
//                                 setStateCommand(hasCoralConditionBatterySide),
//                                 setStateCommand(hasCoralConditionPivotSide),
//                                 () -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE),
//                         Commands.either(
//                             setStateCommand(hasAlgaeConditionBatterySide),
//                             setStateCommand(hasAlgaeConditionPivotSide),
//                             () -> drivetrain.decideScoringMode() == ScoringMode.BATTERY_SIDE),
//                         () -> intake.hasCoral()),
//                 setStateCommand(noPieceCondition),
//                 () -> intake.hasCoral());
//     }

//     private CurrentSuperState handleCurrentSuperStateTransition() {
//         return 
//             switch(wantedSuperState){
//                 case IDLE:
//                     yield CurrentSuperState.IDLING;
//                 case INTAKE_CORAL_FROM_GROUND:
//                     yield CurrentSuperState.INTAKE_CORAL_FROM_GROUND;
//                 case SCORE_L1:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_CORAL_L2_BATTERY_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_CORAL_L2_PIVOT_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_CORAL_L3_BATTERY_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_CORAL_L3_PIVOT_SIDE:
//                     yield CurrentSuperState.SCORING_L3_PIVOT_SIDE;
//                 case REMOVE_ALGAE_L2_BATTERY_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case REMOVE_ALGAE_L2_PIVOT_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case REMOVE_ALGAE_L3_BATTERY_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case REMOVE_ALGAE_L3_PIVOT_SIDE:
//                     yield CurrentSuperState.IDLING;
//                 case INTAKE_ALGAE_FROM_GROUND:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_ALGAE_IN_NET:
//                     yield CurrentSuperState.IDLING;
//                 case SCORE_ALGAE_IN_PROCESSOR:
//                     yield CurrentSuperState.IDLING;
//                 case CLIMB:
//                     yield CurrentSuperState.IDLING;
//         };
//     }
    
//     private void applyStates() {
//         switch(currentSuperState) {
//             case IDLING:
//             arm.setWantedArmMode(ArmWantedMode.IDLE);
//             elevator.setWantedElevatorMode(ElevatorWantedMode.IDLE);
//             intake.setWantedIntakeMode(IntakeWantedMode.IDLE);
//             break;
//             case NO_PIECE_TELEOP:
//             break;
//             case HOLDING_CORAL_TELEOP:
//             break;
//             case HOLDING_ALGAE:
//             break;
//             case INTAKE_CORAL_FROM_GROUND:
//             break;
//             case SCORING_L1:
//             break;
//             case SCORING_L2_BATTERY_SIDE:
//             break;
//             case SCORING_L2_PIVOT_SIDE:
//             break;
//             case SCORING_L3_BATTERY_SIDE:
//             break;
//             case SCORING_L3_PIVOT_SIDE:
//             arm.setWantedArmMode(ArmWantedMode.L3_CORAL_PIVOT);
//             elevator.setWantedElevatorMode(ElevatorWantedMode.L3_CORAL_PIVOT);
//             break;
//             case INTAKING_ALGAE_FROM_REEF:
//             break;
//             case INTAKING_ALGAE_FROM_GROUND:
//             break;
//             case SCORING_ALGAE_IN_NET:
//             break;
//             case SCORING_ALGAE_IN_PROCESSOR:
//             break;
//             case CLIMBING:
//             break;
//         }
        
//     }
// }
