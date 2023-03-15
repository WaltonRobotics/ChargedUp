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
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2).withName("SS-Auto-TopCone");
        // var placeCmd = Commands.parallel(
        //     Commands.waitSeconds(.5).andThen(elev.toHeight(SuperState.TOPCONE.elev.height)), 
        //     tilt.toAngle(SuperState.TOPCONE.tilt.angle),
        //     Commands.waitSeconds(1).andThen(wrist.toAngle(SuperState.TOPCONE.wrist.angle))
        // ).withTimeout(3);
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConePark);

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(1),
            ssResetCmd,
            pathCmd,
            new AutoBalance(swerve, true).withName("autobalance")
        );
    }

    public static CommandBase oneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var printBeginCmd = Commands.print("============ONE_CONE_BACK BEGIN============");
        // var dumbReset = Commands.parallel(
        //     elev.toHeight(SuperState.SAFE.elev.height),
        //     tilt.toAngle(SuperState.SAFE.tilt.angle),
        //     wrist.toAngle(SuperState.SAFE.wrist.angle)
        // );
        var placeCmd = Commands.parallel(
            Commands.waitSeconds(.5)
                .andThen(elev.toHeight(SuperState.TOPCONE.elev.height)),
            tilt.toAngle(SuperState.TOPCONE.tilt.angle),
            Commands.waitSeconds(1)
                .andThen(wrist.toAngle(SuperState.TOPCONE.wrist.angle))
        ).withTimeout(3);
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
        var placeCmd = Commands.parallel(
            Commands.waitSeconds(.65).andThen(elev.toHeight(SuperState.TOPCUBE.elev.height)), 
            tilt.toAngle(SuperState.TOPCUBE.tilt.angle),
            Commands.waitSeconds(1).andThen(wrist.toAngle(SuperState.TOPCUBE.wrist.angle))
        ).withTimeout(3);
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var backCmd = swerve.driveOneDirection(true,3).withTimeout(2.0);
        var strafeCmd = swerve.driveSide(false).withTimeout(1.0);

        return Commands.sequence(
            superstructure.smartReset(),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(0.75),
            ssResetCmd,
            backCmd,
            strafeCmd,
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
