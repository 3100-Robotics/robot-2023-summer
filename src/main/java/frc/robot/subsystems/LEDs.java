package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * this class allows you to control the leds on the robot in many ways
 */
public class LEDs extends SubsystemBase {

    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    private int cubes = 0;

    /**
     * constructs the leds class
     */
    public LEDs() {
        // create an addressable led object and a buffer for it
        led = new AddressableLED(LEDConstants.PWMPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        // set the length of the strip and give it data
        led.setLength(LEDConstants.length);
        led.setData(ledBuffer);
        led.start();
    }

    /**
     * set the color of the entire strip to a rgb color
     */
    public void setColorRGB(int r, int g, int b) {
        // loop through the buffer and set each led to the desired color
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }

        led.setData(ledBuffer);
    }

    /**
     * creates a command to set the entire strip to a rgb color
     * @return the generated command
     */
    public Command setColorRGBCommand(int r, int g, int b) {
        return this.runOnce(() -> setColorRGB(r, g, b));
    }

    /**
     * set the color of each led individually
     */
    public void setIndividualColors(int[] r, int[] g, int[] b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r[i], g[i], b[i]);
        }

        led.setData(ledBuffer);
    }

    /**
     * creates a command to increase the counter of cubes shot by 1
     * @return the generated command
     */
    public Command incrementCubeCounter() {
        return this.runOnce(() -> cubes += 1);
    }

    /**
     * updates the leds to show the number of cubes shot
     */
    public void showCubeCounter() {
        // loop through all the leds
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

    /**
     * create a command to show a rgb color for a specified time
     * @param time the amount of time
     * @return the generated command
     */
    public CommandBase showColorTime(int r, int g, int b, double time) {
        return runOnce(() -> setColorRGB(r, g, b))
                .until(() -> false) // to not end automatically
                .withTimeout(time); // to end
    }
}

