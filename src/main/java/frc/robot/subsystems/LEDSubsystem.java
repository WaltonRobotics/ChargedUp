package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.IndicatorLightsK.*;

public class LEDSubsystem {
    private final AddressableLED led = new AddressableLED(kPort);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(kNumLEDs);

    public LEDSubsystem() {
        led.setLength(kNumLEDs);
        led.setData(buffer);
        led.start();
    }

    public enum GamePieceMode {
        CONE,
        CUBE
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

    public void setWhite(){
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 255, 255);
        }
    }

    // cone is 0, cube is 1
    public void handle(int gamePiece) {
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
