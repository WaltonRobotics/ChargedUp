package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.IndicatorLightsK.*;

public class LEDSubsystem {
    private final AddressableLED led = new AddressableLED(kLedPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(kNumLEDs);

    public LEDSubsystem() {
        led.setLength(kNumLEDs);
        led.setData(ledBuffer);
        led.start();
    }

    public enum GamePieceMode {
        CONE,
        CUBE
    }

    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

    public AddressableLED getLED() {
        return led;
    }

    public void setBlue() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 255);
        }
    }

    public void setYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 255, 0);
        }
    }

    public void setRed() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    public void setPurple() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 128, 0, 128);
        }
    }

    public void setGreen() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    public void setOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setWhite(){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 255, 255);
        }
    }

    // cone is 0, cube is 1
    public void handleLED(int gamePiece) {
        GamePieceMode piece = GamePieceMode.values()[gamePiece];
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