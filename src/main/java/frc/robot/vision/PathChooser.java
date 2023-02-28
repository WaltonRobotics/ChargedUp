package frc.robot.vision;

import java.util.EnumMap;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;

public class PathChooser {
    public enum PathOption {

        NOT_A_PATH("choose a different path smh"),
        RED_BUMPY("red left"),
        BLUE_BUMPY("blue right"),
        RED_NON_BUMPY("red right"),
        BLUE_NON_BUMPY("blue left");

        public final String description;

        PathOption(String description) {
            this.description = description;
        }

        @Override
        public String toString() {
            return name() + ": " + description;
        }
    }

    private PathChooser() {
        // utility class, do not create instances!
    }

    // private static EnumMap<PathOption, CommandBase> pathChooserMap =
    // new EnumMap<PathOption, CommandBase>(PathOption.class);
    private static EnumMap<PathOption, List<PathPoint>> pathChooserMap = new EnumMap<PathOption, List<PathPoint>>(
            PathOption.class);
    private static final SendableChooser<PathOption> PathNTChooser = new SendableChooser<PathOption>();

    static {
        SmartDashboard.putData("Path Chooser", PathNTChooser);
    }

    public static void AssignTrajectory(PathOption path, List<PathPoint> trajectory) {
        pathChooserMap.put(path, trajectory);
        PathNTChooser.addOption(path.description, path);
    }

    public static void SetDefaultPath(PathOption path) {
        PathNTChooser.setDefaultOption(path.description, path);
    }

    public static List<PathPoint> GetPath(PathOption path) {
        return pathChooserMap.get(path);
    }

    public static List<PathPoint> GetChosenPath() {
        return GetPath(PathNTChooser.getSelected());
    }
}