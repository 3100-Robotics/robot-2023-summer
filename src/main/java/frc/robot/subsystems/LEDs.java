package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    public LEDs() {
        led = new AddressableLED(LEDConstants.PWMPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        led.setLength(LEDConstants.length);
        led.setData(ledBuffer);
        led.start();
    }

    public void setColorRGB(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }

        led.setData(ledBuffer);
    }

    public void setIndividualColors(int[] r, int[] g, int[] b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r[i], g[i], b[i]);
        }

        led.setData(ledBuffer);
    }


}

