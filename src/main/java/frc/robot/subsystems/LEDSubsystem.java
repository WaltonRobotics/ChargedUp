package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotContainer;

import static frc.robot.Constants.IndicatorLightsK.*;

public class LEDSubsystem {
    private final AddressableLED led = new AddressableLED(kPort);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(kNumLEDs);
    private static boolean isCone;

    public LEDSubsystem() {
        led.setLength(kNumLEDs);
        led.setData(buffer);
        led.start();
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    public AddressableLED get() {
        return led;
    }

    public void setBlue() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 255);
        }
    }

    public void setYellow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 255, 0);
        }
        isCone = true;
    }

    public void setRed() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 0, 0);
        }
    }

    public void setPurple() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 128, 0, 128);
        }
        isCone = false;
    }

    public void setGreen() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 255, 0);
        }
    }

    public void setOff() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setWhite() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 255, 255);
        }
    }

    public static boolean getHasCone() {
        return isCone;
    }

    // cone is 0, cube is 1
    public void handle(int gamePiece) {
        RobotContainer.GamePieceMode piece = RobotContainer.GamePieceMode.values()[gamePiece];
        switch (piece) {
            case CONE:
                setYellow();
                break;
            case CUBE:
                setPurple();
                break;
            default:
                setWhite();
        }
    }
}
