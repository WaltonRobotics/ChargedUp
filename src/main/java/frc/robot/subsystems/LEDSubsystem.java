package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LedK.*;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_leds = new AddressableLED(kPort);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kNumLeds);

    private final Timer m_ledStateTimer = new Timer();
    private boolean m_chaseUp = false;
    private boolean m_blinkState = false;
    private int m_blinkCount = 0;
    public boolean isCone = false;

    public LEDSubsystem() {
        double subsysInitBegin = Timer.getFPGATimestamp();
        System.out.println("[INIT] LEDSubsystem Init Begin");
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.setData(m_ledBuffer);
        m_leds.start();

        m_ledStateTimer.reset();

        setDefaultCommand(idle());
        double subsysInitElapsed = Timer.getFPGATimestamp() - subsysInitBegin;
		System.out.println("[INIT] LEDSubsystem Init End: " + subsysInitElapsed + "s");
    }

    private void setIdle() {
        for (int i = 0; i < kNumLeds; i++) {
            int ledIdx = i;
            Color col = Color.kWhite;
            if (ledIdx % 2 == 0 || ledIdx == 0) {
                col = (m_chaseUp) ? kBlue : kRed; // "chase" effect
            } else {
                col = (m_chaseUp) ? kRed : kBlue; // "chase" effect
            }
            m_ledBuffer.setLED(i, col);
            // LedUtils.setPixel(m_ledBuffer, ledIdx, col);
        }
        m_chaseUp = !m_chaseUp;
        m_leds.setData(m_ledBuffer);
    }

    public CommandBase setCube() {
        return run(() ->  { 
            isCone = false;
            Color col = Color.kPurple;
            for (int i = 0; i < kNumLeds; i++) {
                m_ledBuffer.setLED(i, col);
            }
            m_leds.setData(m_ledBuffer);
        });
    }

    public CommandBase setCone() {
        return run(() ->  { 
            isCone = true;
            Color col = Color.kYellow;
            for (int i = 0; i < kNumLeds; i++) {
                m_ledBuffer.setLED(i, col);
            }
            m_leds.setData(m_ledBuffer);
        });
    }

    public CommandBase idle() {
        return runOnce(() -> {
            setIdle();
        })
        .ignoringDisable(true)
        .andThen(Commands.waitSeconds(0.5))
        .repeatedly();
    }

    public CommandBase scoreOk() {
        return runOnce(() -> {
            setIdle();
        })
        .ignoringDisable(true)
        .andThen(Commands.waitSeconds(0.125))
        .repeatedly()
        .withTimeout(2);
    }

    public CommandBase grabOk() {
        var init = runOnce(() -> {
            m_ledStateTimer.restart();
            m_blinkState = true;
            m_blinkCount = 0;
        })
        .ignoringDisable(true);
        var cycle = run(() -> {
            if (m_ledStateTimer.advanceIfElapsed(kBlinkPeriod)) {
                Color col = m_blinkState ? kBlinkOnColor : kBlinkOffColor;
                for (int i = 0; i < kNumLeds; i++) {
                    m_ledBuffer.setLED(i, col);
                }
                m_leds.setData(m_ledBuffer);
                m_blinkCount++;
                m_blinkState = !m_blinkState;
            }
        })
        .ignoringDisable(true)
        .until(() -> m_blinkCount >= kBlinkCount * 2);

        return init.andThen(cycle)
        .ignoringDisable(true);
    }
}
