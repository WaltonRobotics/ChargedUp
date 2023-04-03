package frc.robot.auton;

import java.util.EnumMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        DO_NOTHING("do nothing"),
        CONE_ONE_HALF_PARK("1.5 - cone w/ balance"),
        CONE_BACK_PARK("1 - cone w/ balance"),
        CUBE_BACK_PARK("1 - cube w/ balance"),
        TWO_ELEMENT("2 - cone, cube w/o balance)"),
        TWO_ELEMENT_BUMPY("2 - cone, cube w/o balance over bump"),
        TWO_ELEMENT_PARK("2 - cone, cube w/ balance"),
        TWO_ELEMENT_PARK_ALT("2 - cube, cone w/ balance"),
        TWO_POINT_FIVE("3 - cone, cube, cone"),
        CUBE_ONE_HALF_PARK("1.5 - cube w/ balance"),     
        ONE_CONE_OUT("1 - cone w/o balance"),   
        
        //NOT IN USE
        
        ONE_CONE_PARK("NOT IN USE: drop cone then park"),
        CONE_BACK("NOT IN USE: drop cone back"),
        ONE_CUBE_AROUND("NOT IN USE: drop cube then go around and park"),
        THREE_PIECE("NOT IN USE: three piece scoring");
        

        public final String description;

        AutonOption(String description) {
            this.description = description;
        }

        @Override
        public String toString() {
            return name() + ": " + description;
        }
    }

    private AutonChooser() {
        // utility class, do not create instances!
    }

    private static EnumMap<AutonOption, CommandBase> autonChooserMap =
        new EnumMap<>(AutonOption.class);
    private static EnumMap<AutonOption, Optional<Pose2d>> autonInitPoseMap = 
        new EnumMap<>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        SmartDashboard.putData("Auton Chooser", autonNTChooser);
    }

    public static void AssignAutonCommand(AutonOption auton, CommandBase command, Pose2d holonomicStartPose) {
        autonChooserMap.put(auton, command);
        autonInitPoseMap.put(auton, Optional.ofNullable(holonomicStartPose));
        
        autonNTChooser.addOption(auton.description, auton);
    }

    public static void AssignAutonCommand(AutonOption auton, CommandBase command) {
        AssignAutonCommand(auton, command, null);
    }

    public static void SetDefaultAuton(AutonOption auton) {
        autonNTChooser.setDefaultOption(auton.description, auton);
    }

    public static CommandBase GetAuton(AutonOption auton) {
        return autonChooserMap.computeIfAbsent(auton, a -> 
            Commands.print("========WARNING: Empty Auton!!!========")
            .withName("InvalidAuton")
        );
    }

    public static Optional<Pose2d> GetAutonInitPose(AutonOption auton) {
        return autonInitPoseMap.computeIfAbsent(auton, a -> Optional.empty());
    }

    public static CommandBase GetChosenAutonCmd() {
        return GetAuton(autonNTChooser.getSelected());
    }

    public static Optional<Pose2d> GetChosenAutonInitPose() {
        return GetAutonInitPose(autonNTChooser.getSelected());
    }
}