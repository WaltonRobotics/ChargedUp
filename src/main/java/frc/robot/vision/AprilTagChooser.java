package frc.robot.vision;

import java.util.EnumMap;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;

public class AprilTagChooser {
    public enum AprilTagOption {
    
       NOT_A_TAG("no tag selected!!!"),
       TAG_1("tag 1"),
       TAG_2("tag 2"),
       TAG_3("tag 3"),
       TAG_6("tag 6"),
       TAG_7("tag 7"),
       TAG_8("tag 8");     

        public final String description;

        AprilTagOption(String description) {
            this.description = description;
        }

        @Override
        public String toString() {
            return name() + ": " + description;
        }
    }

    private AprilTagChooser() {
        // utility class, do not create instances!
    }

    // private static EnumMap<AprilTagOption, CommandBase> AprilTagChooserMap =
    //     new EnumMap<AprilTagOption, CommandBase>(AprilTagOption.class);
    private static EnumMap<AprilTagOption, PathPoint> AprilTagChooserMap =
        new EnumMap<AprilTagOption, PathPoint>(AprilTagOption.class);
    private static final SendableChooser<AprilTagOption> AprilTagNTChooser = new SendableChooser<AprilTagOption>();

    static {
        SmartDashboard.putData("AprilTag Chooser", AprilTagNTChooser);
    }

    public static void AssignPoint(AprilTagOption tag, PathPoint point) {
        AprilTagChooserMap.put(tag, point);
        AprilTagNTChooser.addOption(tag.description, tag);
    }

    public static void SetDefaultAprilTag(AprilTagOption tag) {
        AprilTagNTChooser.setDefaultOption(tag.description, tag);
    }

    public static PathPoint GetAprilTag(AprilTagOption tag) {
        return AprilTagChooserMap.get(tag);
    }

    public static PathPoint GetChosenAprilTag() {
        return GetAprilTag(AprilTagNTChooser.getSelected());
    }
}