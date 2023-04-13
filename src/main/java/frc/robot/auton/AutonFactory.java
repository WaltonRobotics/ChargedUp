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
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    public static Translation2d position = new Translation2d();

    public static CommandBase testAuto(SwerveSubsystem swerve) {
        var driveCmd = swerve.getPPSwerveAutonCmd(Paths.PPPaths.oneConePark);
        return driveCmd;
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

    // public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    //         ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
    //     var clawCmd = claw.release().withName("ClawRelease");
    //     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
    //     var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.oneConePark);

    //     return Commands.sequence(
    //             tilt.autoHome().alongWith(elev.autoHome()).withTimeout(1.5),
    //             placeCmd.asProxy(),
    //             clawCmd.asProxy(),
    //             Commands.waitSeconds(.5),
    //             ssResetCmd.asProxy(),
    //             pathCmd.asProxy(),
    //             new AutoBalance(swerve, true).withName("autobalance"));
    // }

    public static CommandBase coneOneHalfPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
            ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalf);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");

        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);

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

    public static CommandBase coneOneHalfBumpy(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.coneOneHalfBumpy);
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);

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
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.cubeOneHalf);
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);

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
            ),
            swerve.nowItsTimeToGetFunky().asProxy()
        );
    }

    public static CommandBase cubeOneHalfBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.cubeOneHalfBump);
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);

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
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
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
                    cubePlaceCmd.asProxy().withTimeout(1.7)    //cube throw
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

    public static CommandBase twoElementPark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle);
        var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle2);
        var releaseCmd = claw.release().andThen(Commands.waitSeconds(.25));
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
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
                    cubePlaceCmd.asProxy().withTimeout(1.7)    //cube throw
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
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
        var groundPickUp2 = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
        var placeCmd2 = superstructure.toStateAuton(SuperState.TOPCONE);
        var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);
        var ssResetCmd5 = superstructure.toStateAuton(SuperState.SAFE);

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
            ssResetCmd.asProxy().withTimeout(2.0),
            claw.grab().asProxy(),
            // path after place timeout
            Commands.parallel(
                //path while going to SAFE
                Commands.sequence(
                    Commands.waitSeconds(.20), 
                    pathCmd.asProxy()
                ),

                Commands.sequence(
                    Commands.waitSeconds(1.5),  //Time before pickup
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(2.5),  //time before SAFE
                    // ssResetCmd2.asProxy(), //SAFE
                    // Commands.waitSeconds(1.0),  //time before cube throw
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
                    groundPickUp.asProxy(), //PICKUP
                    Commands.waitSeconds(2.5),  //time before SAFE
                    // ssResetCmd2.asProxy(), //SAFE
                    // Commands.waitSeconds(1),  //time before cube throw
                    lowCubePlaceCmd.asProxy().withTimeout(1.5)    //cube throw
                )
        ),
        Commands.parallel(
            ssResetCmd5.asProxy().withTimeout(1.8),
            Commands.waitSeconds(.5).andThen(claw.grab().asProxy())
        ));
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
        var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
        var groundPickUp2 = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
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
                ssResetCmd4.asProxy() //SAFE
            )
        ));
    }

//     public static CommandBase twoPointFiveBumpy(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
//     ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
//     var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
//     var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
//     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
//     var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEleBump);
//     var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
//     var releaseCmd = claw.release().andThen(Commands.waitSeconds(.275));
//     var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
//     var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
//     var groundPickUp2 = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
//     var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
//     var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);

//     return Commands.sequence(
//     //reset
//     Commands.parallel(
//         tilt.autoHome().asProxy(),
//         elev.autoHome().asProxy()
//     ).withTimeout(1.5),

//     placeCmd.asProxy().withTimeout(1.65),    //to top cone
//     releaseCmd.asProxy(),   //release claw
    
//     //SAFE
//     Commands.parallel(
//         ssResetCmd.asProxy().withTimeout(1.8),
//         claw.grab().asProxy(),
//         // path after place timeout
//         Commands.parallel(
//             //path while going to SAFE
//             Commands.sequence(
//                 Commands.waitSeconds(.10), 
//                 pathCmd.asProxy()
//             ),

//             Commands.sequence(
//                 Commands.waitSeconds(1.40),  //Time before pickup
//                 groundPickUp.asProxy(), //PICKUP
//                 Commands.waitSeconds(0.5),  //time before SAFE
//                 ssResetCmd2.asProxy(), //SAFE
//                 Commands.waitSeconds(.05),  //time before cube throw
//                 cubePlaceCmd.asProxy().withTimeout(1.7)    //cube throw
//             )
//         )
//     ),

//     //SAFE
//     Commands.parallel(
//         ssResetCmd3.asProxy().withTimeout(1.5),
//         claw.grab().asProxy(),

//         Commands.sequence(
//             Commands.waitSeconds(.10),   //time before path
//             path2Cmd.asProxy() // path to pick up :D
//         ),

//         Commands.sequence(
//             Commands.waitSeconds(2.0), // prob will change later ;-;
//             groundPickUp2.asProxy(),
//             Commands.waitSeconds(.6),  //time before SAFE
//             ssResetCmd4.asProxy() //SAFE
//         )
//     ));
// }


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

        
    public static CommandBase onePointFiveBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
                .withTimeout(2.0);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
        var groundCmd = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2.5);
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

    public static CommandBase twoBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
                .withTimeout(2.0);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
        var groundCmd = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2.5);
        // var grabCmd = claw.grab().andThen(Commands.waitSeconds(.29));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
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
                ssResetCmd3.asProxy().withTimeout(1.5),
                claw.grab().asProxy()
            )
        );
    }

    public static CommandBase twoPointFiveBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
        ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
        var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withTimeout(1.5);
        var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE)
                .withTimeout(2.0);
        var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy);
        var groundCmd = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2.5);
        // var grabCmd = claw.grab().andThen(Commands.waitSeconds(.29));
        var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
        var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
        var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
        var pathCmd2 = swerve.getPPSwerveAutonCmd(PPPaths.twoPointFiveBumpy2);
        var groundCmd2 = superstructure.toStateAuton(SuperState.GROUND_PICK_UP).withTimeout(2.5);
        var ssResetCmd4 = superstructure.toStateAuton(SuperState.SAFE);

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
                    // Commands.waitSeconds(1),
                    Commands.waitSeconds(3.6),
                    cubePlaceCmd.asProxy().withTimeout(1.75)
                )
            ),
            Commands.parallel(
                ssResetCmd2.asProxy(),
                Commands.waitSeconds(.2).andThen(pathCmd2.asProxy()),
                Commands.sequence(
                    Commands.waitSeconds(2.5),
                    groundCmd2.asProxy()
                )
            ),
            ssResetCmd4.asProxy()
        );
    }

    // public static CommandBase twoElementBump(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    //     ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
    //     var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
    //     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
    //     var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEleBump);
    //     // var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEle2);
    //     var releaseCmd = claw.release().andThen(Commands.waitSeconds(.3));
    //     var ssResetCmd2 = superstructure.toStateAuton(SuperState.SAFE);
    //     var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
    //     var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);
    
    //     return Commands.sequence(
    //         Commands.parallel(
    //             tilt.autoHome().asProxy(),
    //             elev.autoHome().asProxy()
    //         ).withTimeout(1.0),
    //         placeCmd.asProxy().withTimeout(1.75),
    //         releaseCmd.asProxy(),
    //         // // move to safe state and prepare to move + autoGrab
    //         Commands.parallel(
    //             ssResetCmd.asProxy().withTimeout(1.5),
    //             claw.grab().asProxy(),
    //             // path after place timeout
    //             Commands.deadline(
    //                 Commands.sequence(
    //                     Commands.waitSeconds(1.75), //possible wait until safe
    //                     pathCmd.asProxy()
    //                 ),
    //                 // autograb during path 
    //                 Commands.sequence(
    //                     Commands.waitSeconds(1.25), 
    //                     groundPickUp.asProxy(), //down
    //                     Commands.waitSeconds(1.85), 
    //                     ssResetCmd2.asProxy() //up
    //                 )
    //             )
    //         ),
    //         cubePlaceCmd.asProxy().withTimeout(2.0), //maybe place in parallel w/ wait
    
    //         Commands.parallel(
    //             ssResetCmd3.asProxy().withTimeout(1.5),
    //             claw.grab().asProxy()
    //             //path to balance timeout
    //             // Commands.sequence(
    //             //     Commands.waitSeconds(1.0)
    //                 // path2Cmd.asProxy()
    //                 // swerve.nowItsTimeToGetFunky(false).asProxy()
    //             // )
    //         )
    //     );
    // }

    public static CommandBase oneMeter(SwerveSubsystem swerve) {
        return new RunCommand(()-> 
            swerve.drive(1, 0, 0, false, false)).withTimeout(1);
    }
}


        // public static CommandBase twoElementParkAlt(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw,
    //     ElevatorSubsystem elev, TiltSubsystem tilt, WristSubsystem wrist) {
    //     var cubePlaceCmd = superstructure.cubeTossTop(claw, true);
    //     var placeCmd = superstructure.toStateAuton(SuperState.TOPCONE).withName("SS-Auto-TopCone");
    //     var ssResetCmd = superstructure.toStateAuton(SuperState.SAFE).withName("SS-Auto-Safe");
    //     var pathCmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEleAlt);
    //     var path2Cmd = swerve.getPPSwerveAutonCmd(PPPaths.twoEleAlt2);
    //     var releaseCmd = claw.release().andThen(Commands.waitSeconds(.3));
    //     var groundPickUp = superstructure.toStateAuton(SuperState.GROUND_PICK_UP);
    //     var ssResetCmd3 = superstructure.toStateAuton(SuperState.SAFE);

    //     return Commands.sequence(
    //         //reset
    //         Commands.parallel(
    //             tilt.autoHome().asProxy(),
    //             elev.autoHome().asProxy()
    //         ).withTimeout(1.25),

    //         cubePlaceCmd.asProxy().withTimeout(1.55),    //to top cube
    //         //SAFE
    //         Commands.parallel(
    //             ssResetCmd.asProxy().withTimeout(1.75),
    //             claw.grab().asProxy(),

    //             // path after place timeout
    //             Commands.parallel(
    //                 //path while going to SAFE
    //                 Commands.sequence(
    //                     Commands.waitSeconds(.15), 
    //                     pathCmd.asProxy()
    //                 ),
    //                 // autograb during path 
    //                 Commands.sequence(
    //                     Commands.waitSeconds(1.45),  //Time before pickup
    //                     groundPickUp.asProxy(), //PICKUP
    //                     Commands.waitSeconds(2.5),  //time before cone
    //                     placeCmd.asProxy().withTimeout(1.675),    //to top cone
    //                     releaseCmd.asProxy(),   //release claw
    //                     Commands.waitSeconds(.1)
    //                 )
    //             )
    //         ),

    //         //SAFE
    //         Commands.parallel(
    //             ssResetCmd3.asProxy().withTimeout(1.5),
    //             claw.grab().asProxy(),

    //             Commands.sequence(
    //                 Commands.waitSeconds(.975),   //time before balance
    //                 path2Cmd.asProxy(), //path to balance
    //                 swerve.nowItsTimeToGetFunky().asProxy()
    //             )
    //         )
    //     );
    // }
