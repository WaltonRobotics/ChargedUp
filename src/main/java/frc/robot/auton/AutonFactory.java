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
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe");
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
        var groundPickUp1Cmd = superstructure.toState(SuperState.GROUND_PICK_UP).withTimeout(2);
        var groundPickUp2Cmd = superstructure.toState(SuperState.GROUND_PICK_UP).withTimeout(2);
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
        var releaseCmd = superstructure.releaseClaw();
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var ssResetCmd2 = superstructure.toState(SuperState.SAFE).withName("SS-Auto-Safe");
        var groundPickUp = superstructure.toState(SuperState.GROUND_PICK_UP);

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            releaseCmd,
            Commands.waitSeconds(.6),
            ssResetCmd,
            Commands.deadline(
                pathCmd,
                Commands.waitSeconds(1).andThen(groundPickUp)
            ),
            ssResetCmd2.alongWith(new AutoBalance(swerve, false))
        );
    }

    public static CommandBase simpleOneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3);
        var clawCmd = claw.release()
            .andThen(Commands.waitSeconds(0.75))
            .withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            ssResetCmd,
            new AutoBalance(swerve, true)

        ).withName("OneConeBack");
    }


    public static CommandBase cubeBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCUBE);
        var ssResetCmd = superstructure.toState(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.backPark);
        var rotateCmd = swerve.rotate180();

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            ssResetCmd,
            pathCmd,
            rotateCmd,
            new AutoBalance(swerve, true)
        ).withName("OneConeBack");
    }

    public static CommandBase coneBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5);
        var clawCmd = claw.release()
            .andThen(Commands.waitSeconds(0.5))
            .withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.backPark);
        var rotateCmd = swerve.rotate180();

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            ssResetCmd,
            pathCmd,
            new AutoBalance(swerve, false)
        ).withName("OneConeBack");
    }
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
}
