package frc.robot.commands;

import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private final GenericEntry nte_translation, nte_strafe, nte_rotation, nte_robotCentric;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        nte_translation = DashboardManager.addTabItem("TeleSwerve", "Translation", 0.0);
        nte_strafe = DashboardManager.addTabItem("TeleSwerve", "Strafe", 0.0);
        nte_rotation = DashboardManager.addTabItem("TeleSwerve", "Rotation", 0.0);
        nte_robotCentric = DashboardManager.addTabItem("TeleSwerve", "RobotCentric", false);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        nte_translation.setDouble(translationVal);
        nte_strafe.setDouble(strafeVal);
        nte_rotation.setDouble(rotationVal);
        nte_robotCentric.setBoolean(robotCentricSup.getAsBoolean());

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
        );
    }
}
