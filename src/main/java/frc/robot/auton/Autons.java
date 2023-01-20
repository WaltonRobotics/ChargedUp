// package frc.robot.auton;

// import frc.robot.auton.autoncommands.SwerveTrajectoryCommand;
// import static frc.robot.auton.Paths.UTest.uPath;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public enum Autons{

//     DO_NOTHING("Do Nothing", new SequentialCommandGroup(
//     )),

//     TEST_PATH("Autobalance path", new SequentialCommandGroup(
//             new SwerveTrajectoryCommand(uPath)
//     ));

//     private final String description;
//     private final CommandBase commandGroup;

//     Autons(String description, CommandBase commandGroup) {
//         this.description = description;
//         this.commandGroup = commandGroup;
//     }

//     public String getDescription() {
//         return description;
//     }

//     public CommandBase getCommandGroup() {
//         return commandGroup;
//     }

//     @Override
//     public String toString() {
//         return name() + ": " + description;
//     }
// }