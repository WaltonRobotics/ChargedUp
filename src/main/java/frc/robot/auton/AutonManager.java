package frc.robot.auton;

import java.util.EnumMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonManager {
    public enum AutonOption{

        DO_NOTHING("Do Nothing"),
        MOVE_FORWARD("Move forward 1 meter"),
        MOVE_DIAGONAL("Move diagonal"),
        ROTATE_MOVE("Rotate 90 degrees and strafe right 1.5 meters"),
        ROTATE("Rotate 90 degrees"),
        MOVE_FORWARD_1_METER("Move forward 1 meter (wpilib)"),
        ROTATE_90("Rotate 90 degrees (wpilib)"),
        MOVE_DIAGONALLY("Move diagonal (wpilib)");
    
        private final String description;
    
        AutonOption(String description) {
            this.description = description;
        }
    
        public String getDescription() {
            return description;
        }
    
        @Override
        public String toString() {
            return name() + ": " + description;
        }
    }

    private AutonManager() {
        // utility class, do not create instances!
    }

    private static EnumMap<AutonOption, CommandBase> autonChooserMap = new EnumMap<AutonOption, CommandBase>(AutonOption.class);
    private static final SendableChooser<AutonOption> autonNTChooser = new SendableChooser<AutonOption>();

    static {
        for (var option : AutonOption.values()) {
            autonNTChooser.addOption(option.getDescription(), option);
        }
        SmartDashboard.putData("Auton Chooser", autonNTChooser);
    }

    public static void SetAutonCommand(AutonOption auton, CommandBase command) {
        autonChooserMap.put(auton, command);

    }

    public static void SetDefaultAuton(AutonOption auton) {
        autonNTChooser.setDefaultOption(auton.getDescription(), auton);
    }

    public static CommandBase GetAuton(AutonOption auton) {
        return autonChooserMap.computeIfAbsent(auton, a -> Commands.none().withName("InvalidAuton"));
    }

    public static CommandBase GetChosenAuton() {
        return GetAuton(autonNTChooser.getSelected());
    }
}