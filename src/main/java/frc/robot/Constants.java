// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class driveConstants {
    public static final double balanceP = 0.05;
    public static final double balanceI = 0;
    public static final double balanceD = 0;

    public static final double ROBOT_MASS = 45.35924; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
  }

  public static class cuberConstants {
    public static final int angleMotorPort = 9;
    public static final int leftShooterPort = 10;
    public static final int rightShooterPort = 11;

    public static final double angleP = 0.1;
    public static final double angleI = 0;
    public static final double angleD = 0;

    public enum angles {
      low,
      high,
      mid,
    }
  }
  
  public static class visionConstants {
    public static final double maxHeight = 9.5;
    public static final double[] heightDiffs = {0, 19, 29};
    public static final double g = 32;

    public static final Transform3d robotToFrontCam =
            new Transform3d(
                    new Translation3d(0.222, 0.238, 0),
                    new Rotation3d(
                            60, 0,
                            0));

    public static final Transform3d robotToBackCam =
            new Transform3d(
                    new Translation3d(-0.222, 0.238, 0),
                    new Rotation3d(
                            -60, 0,
                            0));
  }

  public static class LEDConstants {
    public static final int PWMPort = 0;
    public static final int length = 60;
  }
}
