package frc.robot.auton;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auton.Paths.PPPaths;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;

import java.util.HashMap;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    public static Translation2d position = new Translation2d();

    public static CommandBase testAuto(SwerveSubsystem swerve){
        var driveCmd = swerve.getPPSwerveAutonCmd(Paths.PPPaths.oneConePark);
        return driveCmd;
    }

    private static CommandBase logAutonState(String auto, double state) {
        return Commands.runOnce(() -> {
            SmartDashboard.putNumber("AutoState - " + auto, state);
        });
    }
   
    public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConePark);

        return Commands.sequence(
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(.5),
            ssResetCmd,
            pathCmd,
            new AutoBalance(swerve, true).withName("autobalance")
        );
    }

    

    public static CommandBase coneOneHalfPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);

        return Commands.sequence(
            // auto home
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            // move to drop gamepiece (inaccurate due to vision overruns, hence timeout)
            placeCmd.withTimeout(2.5),
            // actually drop, and wait 0.6sec to let it fall
            claw.release().asProxy(), Commands.waitSeconds(.4),
            // move to safe state and prepare to move + autoGrab
            ssResetCmd,
            Commands.deadline(
                pathCmd,
                Commands.waitSeconds(1).andThen(groundPickUp).andThen(Commands.waitSeconds(1.4).andThen(ssResetCmd2))
                ),
            swerve.nowItsTimeToGetFunky()
        );
    }

    public static CommandBase twoElementPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        logAutonState("twoElem", -1).schedule();
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.5));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
            logAutonState("twoElem", 0),
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            logAutonState("twoElem", 1),
            // // move to drop gamepiece (inaccurate due to vision overruns, hence timeout)
            placeCmd.withTimeout(2.5),
            logAutonState("twoElem", 2),
            releaseCmd,
            logAutonState("twoElem", 3),
            // // move to safe state and prepare to move + autoGrab
            Commands.parallel(
                ssResetCmd,
                claw.grab()
            ),
            ssResetCmd.alongWith(claw.grab()),
            logAutonState("twoElem", 4),
            Commands.deadline(
                pathCmd,
                Commands.sequence(
                    logAutonState("twoElem", 4.1),
                    Commands.waitSeconds(1),
                    logAutonState("twoElem", 4.2),
                    groundPickUp,
                    logAutonState("twoElem", 4.3),
                    Commands.waitSeconds(1.85),
                    logAutonState("twoElem", 4.4),
                    ssResetCmd2,
                    logAutonState("twoElem", 4.5)
                )
            ),
            logAutonState("twoElem", 5),
            cubePlaceCmd.withTimeout(2.5),
            logAutonState("twoElem", 6),

            ssResetCmd3.alongWith(claw.grab()),
            logAutonState("twoElem", 7)

        );
    }



    public static CommandBase cubeBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.backPark);

        return Commands.sequence(
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            placeCmd,
            ssResetCmd,
            pathCmd,
            swerve.nowItsTimeToGetFunky()
        ).withName("OneConeBack");
    }

    public static CommandBase coneBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.backPark);

        return Commands.sequence(
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            placeCmd,
            claw.release().asProxy(), Commands.waitSeconds(.45),
            ssResetCmd,
            pathCmd,
            swerve.nowItsTimeToGetFunky()
        ).withName("OneConeBack");
    }
    
    // public static CommandBase oneCubeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var placeCmd = superstructure.toStateAuton(SuperState.TOPCUBE).andThen(claw.release()).withName("SS-Auto-TopCone");
    //     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneCubePark);

    //     return Commands.sequence(
    //         tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
    //         placeCmd,
    //         ssResetCmd,
    //         pathCmd,
    //         new AutoBalance(swerve, false)
    //     );
    // }


        // public static CommandBase simpleOneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(3);
    //     var clawCmd = claw.release()
    //         .andThen(Commands.waitSeconds(0.75))
    //         .withName("ClawRelease");
    //     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
    //         .withTimeout(2)
    //         .withName("SS-Auto-Safe");

    //     return Commands.sequence(
    //         tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
    //         placeCmd,
    //         clawCmd,
    //         ssResetCmd,
    //         new AutoBalance(swerve, true)

    //     ).withName("OneConeBack");
    // }

    
        // public static CommandBase threePiece(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var placeCone1Cmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2).andThen(claw.release()).withName("SS-Auto_TopCone");
    //     var placeCone2Cmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2).andThen(claw.release()).withName("SS-Auto_TopCone");
    //     var reset1Cmd = superstructure.toStateAuton(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var reset2Cmd = superstructure.toStateAuton(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var reset3Cmd = superstructure.toStateAuton(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var groundPickUp1Cmd = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2);
    //     var groundPickUp2Cmd = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2);
    //     var placeCubeCmd = superstructure.toStateAuton(SuperState.TOPCUBE).withTimeout(2).andThen(claw.release()).withName("SS-Auto-TopCube");

    //     return Commands.sequence(
    //         tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
    //         placeCone1Cmd,
    //         Commands.waitSeconds(1),
    //         reset1Cmd,
    //         swerve.getPPSwerveAutonCmd(PPPaths.threePiece1),
    //         groundPickUp1Cmd,
    //         swerve.getPPSwerveAutonCmd(PPPaths.threePiece2),
    //         placeCubeCmd,
    //         Commands.waitSeconds(1),
    //         reset2Cmd,
    //         swerve.getPPSwerveAutonCmd(PPPaths.threePiece3),
    //         groundPickUp2Cmd,
    //         swerve.getPPSwerveAutonCmd(PPPaths.threePiece4),
    //         placeCone2Cmd,
    //         Commands.waitSeconds(1),
    //         reset3Cmd
    //     );
    // }

}
