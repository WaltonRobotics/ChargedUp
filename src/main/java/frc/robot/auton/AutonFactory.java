package frc.robot.auton;

import edu.wpi.first.math.geometry.Translation2d;
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
   
    public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConePark);

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(.5),
            ssResetCmd,
            pathCmd,
            new AutoBalance(swerve, true).withName("autobalance")
        );
    }

    public static CommandBase twoElementPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var conePlaceCmd = superstructure.toState(SuperState.TOPCONE).withName("SS-Auto-TopCone").andThen(claw.release());
        var cubePlaceCmd = superstructure.toState(SuperState.TOPCUBE).withName("SS-Auto-TopCube");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe");
        var ssResetCmd2 = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe2");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoElement);
    
        return Commands.sequence(
            superstructure.smartReset(),
            cubePlaceCmd,
            Commands.waitSeconds(.75),
            ssResetCmd,
            pathCmd.alongWith(superstructure.toState(SuperState.GROUND_PICK_UP)),
            conePlaceCmd,
            Commands.waitSeconds(.75),
            ssResetCmd2
        );
    }
    public static CommandBase threePiece(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCone1Cmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2).andThen(claw.release()).withName("SS-Auto_TopCone");
        var placeCone2Cmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2).andThen(claw.release()).withName("SS-Auto_TopCone");
        var reset1Cmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var reset2Cmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var reset3Cmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var groundPickUp1Cmd = superstructure.toState(SuperState.GROUND_PICK_UP).withTimeout(2).andThen(claw.autoGrab(false).withTimeout(2));
        var groundPickUp2Cmd = superstructure.toState(SuperState.GROUND_PICK_UP).withTimeout(2).andThen(claw.autoGrab(false).withTimeout(2));
        var placeCubeCmd = superstructure.toState(SuperState.TOPCUBE).withTimeout(2).andThen(claw.release()).withName("SS-Auto-TopCube");

        return Commands.sequence(
            superstructure.smartReset(),
            placeCone1Cmd,
            Commands.waitSeconds(1),
            reset1Cmd,
            swerve.getPPSwerveAutonCmd(PPPaths.threePiece1),
            groundPickUp1Cmd,
            swerve.getPPSwerveAutonCmd(PPPaths.threePiece2),
            placeCubeCmd,
            Commands.waitSeconds(1),
            reset2Cmd,
            swerve.getPPSwerveAutonCmd(PPPaths.threePiece3),
            groundPickUp2Cmd,
            swerve.getPPSwerveAutonCmd(PPPaths.threePiece4),
            placeCone2Cmd,
            Commands.waitSeconds(1),
            reset3Cmd
        );
    }

    public static CommandBase coneOneHalfPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var releaseCmd = claw.release();
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(1.5).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var ssResetCmd2 = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe");

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            releaseCmd,
            Commands.waitSeconds(.6),
            ssResetCmd,
            pathCmd.alongWith(Commands.waitSeconds(.75).andThen(superstructure.toState(SuperState.GROUND_PICK_UP)).withTimeout(PPPaths.coneOneHalf.getTotalTimeSeconds())),
            new AutoBalance(swerve, false).withName("autobalance").alongWith(ssResetCmd2)
        );
    }

    public static CommandBase oneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var printBeginCmd = Commands.print("============ONE_CONE_BACK BEGIN============");
        // var dumbReset = Commands.parallel(
        //     elev.toHeight(SuperState.SAFE.elev.height),  
        //     tilt.toAngle(SuperState.SAFE.tilt.angle),
        //     wrist.toAngle(SuperState.SAFE.wrist.angle)
        // );
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3);
        var clawCmd = claw.release()
            .andThen(Commands.waitSeconds(0.75))
            .withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");

        return Commands.sequence(
            superstructure.smartReset(),
            printBeginCmd,
            placeCmd,
            clawCmd,
            ssResetCmd,
            new AutoBalance(swerve, true)

        ).withName("OneConeBack");
    }

    // public static CommandBase oneConeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
    //     var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
    //     var clawCmd = claw.release().withName("ClawRelease");
    //     var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var backCmd = swerve.driveOneDirection(true).withTimeout(3);
    //     var strafeCmd = swerve.driveSide(true).withTimeout(2);

    //     return Commands.sequence(
    //         placeCmd,
    //         clawCmd,
    //         ssResetCmd,
    //         backCmd,
    //         strafeCmd,
    //         new AutoBalance(swerve, false)
    //     );
    // }

    public static CommandBase oneCubeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCUBE).andThen(claw.release()).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneCubePark);

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            ssResetCmd,
            pathCmd,
            new AutoBalance(swerve, false)
        );
    }

    public static CommandBase dropTopCone(Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = Commands.parallel(
            Commands.waitSeconds(.65).andThen(elev.toHeight(SuperState.TOPCUBE.elev.height)), 
            tilt.toAngle(SuperState.TOPCUBE.tilt.angle),
            Commands.waitSeconds(1).andThen(wrist.toAngle(SuperState.TOPCUBE.wrist.angle))
        ).withTimeout(3);
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(0.75),
            ssResetCmd
        );
    }

    public static CommandBase releaseClaw(TheClaw claw) {
        return claw.release();
    }
}
