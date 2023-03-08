package frc.robot.auton;

import java.util.EnumMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        STRAIGHT_BACK("Straight Back"),
        ONE_CONE_ONE_CUBE("Drop Cone then Cube"),
        ONE_CONE("Drop Cone"),
        TEST_ROT("test"),
        ONE_CONE_PARK("Drop cone then park"),
        ONE_CONE_PARK_EVENTS("Drop cone then park w/ Events"),
        ONE_CONE_PARK_WPILIB("Drop cone then park using wpilib trajectory"),
        ONE_CUBE_ONE_CONE("Drop cube then cone and park w/ events"),
        TWO_CONE_ONE_CUBE("Drop cone then cube then cone and park w/ events"),
        CONE_BUMPER("drop a cone and then drive out (bumper side)"),
        CUBE_BUMPER("drop a cube and then drive out (bumper side)"),
        CONE_NOT_BUMPER("drop a cone and then drive out (non-bumper side)"),
        CUBE_NOT_BUMPER("drop a cube and then drive out (non-bumper side)"),
        RELEASE_CLAW("releases the claw"),
        AUTOBALANCE("autobalance"),
        PURE_DRIVE("Pure Drive");

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
        new EnumMap<AutonOption, CommandBase>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        SmartDashboard.putData("Auton Chooser", autonNTChooser);
    }

    public static void AssignAutonCommand(AutonOption auton, CommandBase command) {
        autonChooserMap.put(auton, command);
        autonNTChooser.addOption(auton.description, auton);
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

    public static CommandBase GetChosenAuton() {
        return GetAuton(autonNTChooser.getSelected());
    }
}