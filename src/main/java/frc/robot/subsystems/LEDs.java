package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    private int cubes = 0;

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

    public Command setColorRGBCommand(int r, int g, int b) {
        return this.runOnce(() -> setColorRGB(r, g, b));
    }

    public void setIndividualColors(int[] r, int[] g, int[] b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r[i], g[i], b[i]);
        }

        led.setData(ledBuffer);
    }

    public Command incrementCubeCounter() {
        return this.runOnce(() -> cubes += 1);
    }

    public void showCubeCounter() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i <= cubes*4) {
                ledBuffer.setRGB(i, 153, 51, 255);
            }
            else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }

        led.setData(ledBuffer);
    }

    public CommandBase showColorTime(int r, int g, int b, double time) {
        return runOnce(() -> setColorRGB(r, g, b))
                .until(() -> false) // to not end automatically
                .withTimeout(time); // to end
    }
}

