package frc.robot.auton;

import java.util.EnumMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        DO_NOTHING("do nothing"),
        CONE_BACK_PARK("1 - cone w/ balance"),
        CUBE_BACK_PARK("1 - cube w/ balance"),
        ONE_POINT_FIVE_ELEMENT_BAL_CHARGE("1.5 - cone w/ balance over charge"),
        TWO_ELEMENT("2 - cone, cube w/o balance"),
        TWO_ELEMENT_BAL_CHARGE("2 - cone, cube w/ balance over charge"),
        TWO_POINT_FIVE("2.5 - cone, cube w/o balance"),
        THREE_ELEMENT("3 - cone, cube, cube w/o balance");

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

    private static EnumMap<AutonOption, Command> autonChooserMap = new EnumMap<>(AutonOption.class);
    private static EnumMap<AutonOption, Optional<Pose2d>> autonInitPoseMap = new EnumMap<>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        SmartDashboard.putData("Auton Chooser", autonNTChooser);
    }

    public static void AssignAutonCommand(AutonOption auton, Command command, Pose2d holonomicStartPose) {
        autonChooserMap.put(auton, command);
        autonInitPoseMap.put(auton, Optional.ofNullable(holonomicStartPose));

        autonNTChooser.addOption(auton.description, auton);
    }

    public static void AssignAutonCommand(AutonOption auton, Command command) {
        AssignAutonCommand(auton, command, null);
    }

    public static void SetDefaultAuton(AutonOption auton) {
        autonNTChooser.setDefaultOption(auton.description, auton);
    }

    public static Command GetAuton(AutonOption auton) {
        return autonChooserMap.computeIfAbsent(auton, a -> Commands.print("========WARNING: Empty Auton!!!========")
                .withName("InvalidAuton"));
    }

    public static Optional<Pose2d> GetAutonInitPose(AutonOption auton) {
        return autonInitPoseMap.computeIfAbsent(auton, a -> Optional.empty());
    }

    public static Command GetChosenAutonCmd() {
        return GetAuton(autonNTChooser.getSelected());
    }

    public static Optional<Pose2d> GetChosenAutonInitPose() {
        return GetAutonInitPose(autonNTChooser.getSelected());
    }
}