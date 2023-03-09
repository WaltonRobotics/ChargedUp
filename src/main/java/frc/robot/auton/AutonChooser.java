package frc.robot.auton;

import java.util.EnumMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
    public enum AutonOption {
        TEST_ROT("test"),
        ONE_CONE_PARK("drop cone then park"),
        DROP_CONE_BACK("drop cone back"),
        // ONE_CONE_AROUND("drop cone then go around and park"),
        ONE_CUBE_AROUND("drop cube then go around and park"),
        AUTOBALANCE("autobalance");

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