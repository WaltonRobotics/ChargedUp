package frc.robot.auton;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auton.Paths.PPPaths;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    public static Translation2d position = new Translation2d();

    public static CommandBase testAuto(SwerveSubsystem swerve) {
        var driveCmd = swerve.getPPSwerveAutonCmd(Paths.PPPaths.oneConePark);
        return driveCmd;
    }

    public static CommandBase oneMeter(SwerveSubsystem swerve) {
        return new RunCommand(()-> 
            swerve.drive(1, 0, 0, false, false)).withTimeout(1);
    }

    public static CommandBase coneDrop(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw, 
            ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");

        return Commands.sequence(
            tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
            placeCmd.asProxy(),
            clawCmd.asProxy(),
            Commands.waitSeconds(.5),
            ssResetCmd.asProxy()
        );
    }

    public static CommandBase coneBackOut(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2.5);
    var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2)
            .withName("SS-Auto-Safe");
    var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConeOut);

    return Commands.sequence(
            tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
            placeCmd.asProxy(),
            claw.release().asProxy(), Commands.waitSeconds(0.45).asProxy(),
            ssResetCmd.asProxy(),
            pathCmd.asProxy()
    );
}
    public static CommandBase oneConeBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2.5);
    var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
    .withTimeout(2)
    .withName("SS-Auto-Safe");
    var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConeBump);

    return Commands.sequence(
    tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
    placeCmd.asProxy(),
    claw.release().asProxy(), Commands.waitSeconds(0.45).asProxy(),
    ssResetCmd.asProxy(),
    pathCmd.asProxy()
    );
}

    public static CommandBase coneBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(2.5);
    var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2);
    var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneBackPark);

    return Commands.sequence(
            tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
            placeCmd.asProxy(),
            claw.release(), Commands.waitSeconds(.45),
            ssResetCmd.asProxy(),
            pathCmd.asProxy(),
            swerve.nowItsTimeToGetFunky().asProxy());
}

    public static CommandBase cubeBackPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    var placeCmd = superstructure.cubeTossTop(claw, true);
    var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2);
    var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.backPark);

    return Commands.sequence(
            tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
            placeCmd.asProxy(),
            ssResetCmd.asProxy(),
            pathCmd.asProxy(),
            swerve.nowItsTimeToGetFunky().asProxy()).withName("OneCubeBack");
}

    public static CommandBase coneOneHalfPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
            ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");

        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);

        return Commands.sequence(
                // auto home
                tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
                // move to drop gamepiece (inaccurate due to vision overruns, hence timeout)
                placeCmd.asProxy().withTimeout(2.5),
                // actually drop, and wait 0.6sec to let it fall
                claw.release().asProxy(), Commands.waitSeconds(.4),
                // move to safe state and prepare to move + autoGrab
                ssResetCmd.asProxy(),
                Commands.deadline(
                        pathCmd.asProxy(),
                        Commands.waitSeconds(1.5).andThen(groundPickUp.asProxy())
                                .andThen(Commands.waitSeconds(1.4).andThen(ssResetCmd2.asProxy()))),
                swerve.nowItsTimeToGetFunky().asProxy());
    }

    public static CommandBase onePointFiveBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
                .withTimeout(2.0);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
        var groundCmd = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP).withTimeout(2.5);
        // var grabCmd = claw.grab().andThen(Commands.waitSeconds(.29));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
                tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
                placeCmd.asProxy(),
                claw.release().asProxy().alongWith(Commands.waitSeconds(.29)),
                Commands.parallel(
                    ssResetCmd.asProxy(),
                    Commands.sequence(
                        Commands.waitSeconds(.2),
                        pathCmd.asProxy()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(2.0),
                        groundCmd.asProxy(),
                        Commands.waitSeconds(2),
                        // grabCmd.asProxy(),
                        ssResetCmd2.asProxy()
                    )
                )
        );
    }

    public static CommandBase coneOneHalfBumpy(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalfBumpy);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);

        return Commands.sequence(
                // auto home
                tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
                // move to drop gamepiece (inaccurate due to vision overruns, hence timeout)
                placeCmd.asProxy().withTimeout(2.5),
                // actually drop, and wait 0.6sec to let it fall
                claw.release().asProxy(), Commands.waitSeconds(.4),
                // move to safe state and prepare to move + autoGrab
                ssResetCmd.asProxy(),
                Commands.deadline(
                        pathCmd.asProxy(),
                        Commands.waitSeconds(1).andThen(groundPickUp.asProxy())
                                .andThen(Commands.waitSeconds(1.4).andThen(ssResetCmd2.asProxy()))),
                swerve.nowItsTimeToGetFunky().asProxy());
    }

    public static CommandBase cubeOneHalfPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.cubeOneHalf);
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);

        return Commands.sequence(
            Commands.parallel(
                tilt.autoHome().asProxy(),
                elev.autoHome().asProxy()
            ).withTimeout(1.5),

            cubePlaceCmd.asProxy().withTimeout(1.55),
            ssResetCmd.asProxy().withTimeout(1.35),
            Commands.parallel(
                pathCmd.asProxy(),
                Commands.sequence(
                    Commands.waitSeconds(4.00),
                    groundPickUp.asProxy(),
                    Commands.waitSeconds(1.1),
                    ssResetCmd2.asProxy()
                )
            ),
            swerve.nowItsTimeToGetFunky().asProxy()
        );
    }

    public static CommandBase cubeOneHalfBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.cubeOneHalfBump);
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);

        return Commands.sequence(
            Commands.parallel(
                tilt.autoHome().asProxy(),
                elev.autoHome().asProxy()
            ).withTimeout(1.5),

            cubePlaceCmd.asProxy().withTimeout(1.55),
            ssResetCmd.asProxy().withTimeout(1.35),
            Commands.deadline(
                pathCmd.asProxy(),
                Commands.sequence(
                    Commands.waitSeconds(4.00),
                    groundPickUp.asProxy()
                )
            )
        );
    }


    public static CommandBase twoElement(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.175));
        var retractCmd = claw.extendFlaps(false);
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
        //reset
        Commands.parallel(
            tilt.autoHome().asProxy(),
            elev.autoHome().asProxy()
        ).withTimeout(1.5),

        placeCmd.asProxy().withTimeout(1.65),    //to top cone
        releaseCmd.asProxy(),   //release claw
        
        //SAFE
        Commands.parallel(
            ssResetCmd.asProxy().withTimeout(1.8),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    Commands.waitSeconds(.10), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(1.40),  //Time before pickup
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(0.5),  //time before SAFE
                    ssResetCmd2.asProxy(), //SAFE
                    Commands.waitSeconds(.05),  //time before cube throw
                    cubePlaceCmd.asProxy().withTimeout(1.7),    //cube throw
                    retractCmd.asProxy()
                )
            )
        ),
            //SAFE
            Commands.parallel(
                ssResetCmd3.asProxy().withTimeout(1.5),
                claw.grab().asProxy()
            )
        );
    }

    public static CommandBase twoBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
                .withTimeout(2.0);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
        var groundCmd = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP).withTimeout(2.5);
        var retractCmd = claw.extendFlaps(false);
        // var grabCmd = claw.grab().andThen(Commands.waitSeconds(.29));
        var tossCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
            tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
            placeCmd.asProxy(),
            claw.release().asProxy().alongWith(Commands.waitSeconds(.275)),
            Commands.parallel(
                ssResetCmd.asProxy(),
                Commands.sequence(
                    Commands.waitSeconds(.2),
                    pathCmd.asProxy()
                ),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    groundCmd.asProxy()
                    // Commands.waitSeconds(1),
                    // grabCmd.asProxy(),
                    // ssResetCmd2.asProxy()
                )
            ),
            Commands.sequence(
                tossCmd.asProxy().withTimeout(1.5),
                retractCmd.asProxy(),
                ssResetCmd3.asProxy().withTimeout(1.5),
                claw.grab().asProxy()
            )
        );
    }

    public static CommandBase twoElementPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle);
        var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle2);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.25));
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var retractCmd = claw.extendFlaps(false);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
        //reset
        Commands.parallel(
            tilt.autoHome().asProxy(),
            elev.autoHome().asProxy()
        ).withTimeout(1.5),

        placeCmd.asProxy().withTimeout(1.75),    //to top cone
        releaseCmd.asProxy(),   //release claw
        
        //SAFE
        Commands.parallel(
            ssResetCmd.asProxy().withTimeout(1.8),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    // Commands.waitSeconds(.10), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(1.40),  //Time before pickup
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(0.5),  //time before SAFE
                    ssResetCmd2.asProxy(), //SAFE
                    Commands.waitSeconds(.15),  //time before cube throw
                    cubePlaceCmd.asProxy().withTimeout(1.7),    //cube throw
                    retractCmd.asProxy()
                )
            )
        ),
            //SAFE
            Commands.parallel(
                ssResetCmd3.asProxy().withTimeout(1.5),
                Commands.waitSeconds(.75).andThen(claw.grab().asProxy()),

                Commands.sequence(
                    Commands.waitSeconds(.975),   //time before balance
                    path2Cmd.asProxy(), //path to balance
                    swerve.nowItsTimeToGetFunky().asProxy()
                )
            )
        );
    }

    public static CommandBase chargeTwoElement(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.chargeTwo);
        var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle2);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.25));
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
        //reset
        Commands.parallel(
            tilt.autoHome().asProxy(),
            elev.autoHome().asProxy()
        ).withTimeout(1.5),

        placeCmd.asProxy().withTimeout(1.75),    //to top cone
        releaseCmd.asProxy(),   //release claw
        
        //SAFE
        Commands.parallel(
            ssResetCmd.asProxy().withTimeout(3.0),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    Commands.waitSeconds(.10), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(4.0),  //Time before pickup ~ need to change
                    groundPickUp.asProxy() //PICKUP
                     //SAFE
                )
            )
        ),
        Commands.parallel(
            Commands.waitSeconds(1.0).andThen(ssResetCmd2.asProxy()),
            path2Cmd.asProxy(), //path to balance
            Commands.sequence(
                Commands.waitSeconds(6),  //time before cube throw ~ change timing
                cubePlaceCmd.asProxy().withTimeout(1.7)  //cube throw
            )
        ),
        ssResetCmd3.asProxy()
        );
    }

    public static CommandBase twoPointFive(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle);
        var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.three);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.275));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var groundPickUp2 = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var retractCmd = claw.extendFlaps(false);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
        //reset
        Commands.parallel(
            tilt.autoHome().asProxy(),
            elev.autoHome().asProxy()
        ).withTimeout(1.5),

        placeCmd.asProxy().withTimeout(1.75),    //to top cone
        releaseCmd.asProxy(),   //release claw
        
        //SAFE
        Commands.parallel(
            ssResetCmd.asProxy().withTimeout(1.8),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    Commands.waitSeconds(.10), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(1.40),  //Time before pickup
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(0.5),  //time before SAFE
                    ssResetCmd2.asProxy(), //SAFE
                    Commands.waitSeconds(.05),  //time before cube throw
                    cubePlaceCmd.asProxy().withTimeout(1.7)    //cube throw
                )
            )
        ),

        //SAFE
        Commands.parallel(
            ssResetCmd3.asProxy().withTimeout(1.5),
            claw.grab().asProxy(),

            Commands.sequence(
                Commands.waitSeconds(.10),   //time before path
                path2Cmd.asProxy() // path to pick up :D
            ),

            Commands.sequence(
                Commands.waitSeconds(2.0), // prob will change later ;-;
                groundPickUp2.asProxy(),
                Commands.waitSeconds(.6),  //time before SAFE
                ssResetCmd4.asProxy(), //SAFE
                retractCmd.asProxy()
            )
        ));
    }

    public static CommandBase twoPointFiveBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.7);
    var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
            .withTimeout(2.0);
    var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
    var groundCmd = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP).withTimeout(2.5);
    var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
    var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
    var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
    var pathCmd2 = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy2);
    var groundCmd2 = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP).withTimeout(2.5);
    var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);
    var retractCmd = claw.extendFlaps(false);

    return Commands.sequence(
        tilt.autoHome().asProxy().alongWith(elev.autoHome().asProxy()).withTimeout(1.5),
        placeCmd.asProxy(),
        claw.release().asProxy().alongWith(Commands.waitSeconds(.25)),
        Commands.parallel(
            ssResetCmd.asProxy(),
            Commands.sequence(
                Commands.waitSeconds(.2),
                pathCmd.asProxy()
            ),
            Commands.sequence(
                Commands.waitSeconds(2.6),
                groundCmd.asProxy(),
                Commands.waitSeconds(2.2),
                ssResetCmd2.asProxy(),
                Commands.waitSeconds(1.0),
                cubePlaceCmd.asProxy().withTimeout(1.55)
            )
        ),
        Commands.parallel(
            ssResetCmd3.asProxy(),
            Commands.waitSeconds(.2).andThen(pathCmd2.asProxy()),
            Commands.sequence(
                Commands.waitSeconds(2.5),
                groundCmd2.asProxy()
            )
        ),
        ssResetCmd4.asProxy(),
        retractCmd.asProxy()
    );
}
    public static CommandBase threeElement(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var lowCubePlaceCmd = superstructure.cubeTossMid(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle);
        var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.three);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.29));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var groundPickUp = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var groundPickUp2 = superstructure.toStateAuton(SuperState.EXTENDED_GROUND_PICK_UP);
        var retractCmd = claw.extendFlaps(false);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd5 = superstructure.toStateAuton(SuperState.SAFE);

        return Commands.sequence(
        //reset
        Commands.parallel(
            tilt.autoHome().asProxy(),
            elev.autoHome().asProxy()
        ).withTimeout(1.5),

        placeCmd.asProxy().withTimeout(1.55),    //to top cone
        releaseCmd.asProxy(),   //release claw
        
        //SAFE
        Commands.parallel(
            ssResetCmd.asProxy().withTimeout(3.0),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    Commands.waitSeconds(.1), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(1.4),  //Time before pickup
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(2.0),  //time before SAFE
                    ssResetCmd2.asProxy(), //SAFE
                    Commands.waitSeconds(.05),  //time before cube throw
                    cubePlaceCmd.asProxy().withTimeout(1.50)    //cube throw
                )
            )
        ),

        //SAFE
        Commands.parallel(
            ssResetCmd3.asProxy().withTimeout(1.5),
            claw.grab().asProxy(),

            Commands.sequence(
                Commands.waitSeconds(.025),   //time before path
                path2Cmd.asProxy() // path to pick up :D
            ),

            Commands.sequence(
                    Commands.waitSeconds(1.5),  //Time before pickup
                    groundPickUp2.asProxy(), //PICKUP
                    Commands.waitSeconds(1.3),  //time before SAFE
                    ssResetCmd4.asProxy(), //SAFE
                    Commands.waitSeconds(.05),  //time before cube throw
                    lowCubePlaceCmd.asProxy().withTimeout(1.5)    //cube throw
                )
        ),
        Commands.parallel(
            ssResetCmd5.asProxy().withTimeout(1.8),
            Commands.waitSeconds(.5).andThen(claw.grab().asProxy())
        ),
        retractCmd.asProxy());
    }
}
