package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;

public class ReverseBalance extends SequentialCommandGroup {

    private CommandBase logBalanceState(double state) {
		return Commands.runOnce(() -> {
			log_balState.accept(state);
		});
	}

    private final double m_rateThreshold = 5.0; //6.0
    private final double m_climbRateTimeout = 1.5;

    private double m_climbingSign = 0.0;
    private final Timer m_climbTimer = new Timer();
    private final DoubleLogger log_balState = WaltLogger.logDouble("Command", "BalanceState");

    public static Trigger m_reverseBalanceTrig;


    public ReverseBalance(SwerveSubsystem swerve) {
        CommandBase oneHopThisTime =
            Commands.run(
                ()-> swerve.drive(-2.75, 0, 0, false, false), swerve)
        .until(()-> Math.abs(swerve.getGyroPitch()) > 14)
        .finallyDo((intr) -> {      
            m_climbingSign = Math.signum(swerve.getGyroPitch());
        });
		
		CommandBase takeItBackNowYall = Commands.run(()-> {
                swerve.drive(-0.65, 0,0, false, false);
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

        m_reverseBalanceTrig = new Trigger(() -> takeItBackNowYall.isFinished());

        addCommands(
            logBalanceState(1),
            oneHopThisTime,
            Commands.runOnce(() -> {
                m_climbTimer.restart();
            }),
            logBalanceState(2),
			takeItBackNowYall,
            logBalanceState(3),
            Commands.runOnce(swerve::xLock),
            logBalanceState(4)
        );
    }
}
