package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NewBalance extends SequentialCommandGroup {

    private CommandBase logBalanceState(int state) {
		return Commands.runOnce(() -> {
			SmartDashboard.putNumber("BalanceState", state);
		});
	}

    private final double m_rateThreshold = 5;
    private final double m_climbRateTimeout = 1.5;

    private double m_climbingSign = 0.0;
    private final Timer m_climbTimer = new Timer();


    public NewBalance(SwerveSubsystem swerve) {

        CommandBase oneHopThisTime = Commands.run(
            ()-> swerve.drive(2.3, 0, 0, false, false), swerve)
        .until(()-> Math.abs(swerve.getGyroPitch()) > 14 )
        .finallyDo((intr) -> {
            m_climbingSign = Math.signum(swerve.getGyroPitch());
        });
		
		CommandBase slideToTheFront = Commands.run(()-> {
            swerve.drive(0.5, 0, 0, false, false);
        }, swerve)
        .until(() -> {
            var curPitchRate = swerve.getFilteredGyroPitchRate();

            if (m_climbingSign == -1) {
                if (curPitchRate > m_rateThreshold) {
                    return m_climbTimer.hasElapsed(m_climbRateTimeout);
                }
            } else {
                if (curPitchRate < m_rateThreshold * -1) {
                    return m_climbTimer.hasElapsed(m_climbRateTimeout);
                }
            }

            return false;
        });
		
		// var takeItBackNowYall = Commands.run(()-> swerve.drive(-0.6, 0, 0, false, false), swerve)
		// .withTimeout(1.175);

        addCommands(
            logBalanceState(1),
            oneHopThisTime,
            Commands.runOnce(() -> {
                m_climbTimer.restart();
            }),
            logBalanceState(2),
			slideToTheFront,
            logBalanceState(3),
            Commands.runOnce(swerve::stopWithX),
            logBalanceState(4)
        );
    }
}
