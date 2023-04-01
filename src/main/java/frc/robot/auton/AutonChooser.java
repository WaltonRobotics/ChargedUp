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
        CONE_ONE_HALF_PARK("cone, pickup, balance"),
        CONE_BACK_PARK("cone, leave, balance"),
        CUBE_BACK_PARK("cube, leave, balance"),
        TWO_ELEMENT("cone-cube-nopark"),
        TWO_ELEMENT_BUMPY("cone-cube-nopark on bumpy side"),
        TWO_ELEMENT_PARK("cone-cube-park"),
        TWO_ELEMENT_PARK_ALT("cube-cone-park"),
        TWO_POINT_FIVE("cone-cube-get-another"),
        CUBE_ONE_HALF_PARK("cube, pickup, balance"),     
        ONE_CONE_OUT("cone, drive out of community"),   
        
        //NOT IN USE
        
        ONE_CONE_PARK("drop cone then park"),
        CONE_BACK("drop cone back"),
        ONE_CUBE_AROUND("drop cube then go around and park"),
        THREE_PIECE("three piece scoring");
        

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