package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import frc.robot.subsystems.Drive;

/**
 * An example command that uses an example subsystem.
 */
public class drive extends CommandBase
{

  private final Drive swerve;
  private final DoubleSupplier  vX, vY, heading;
  private final boolean isOpenLoop;
  private final BooleanSupplier isFieldOriented;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public drive(Drive swerve, DoubleSupplier vX, DoubleSupplier vY,
               DoubleSupplier heading, BooleanSupplier isFieldOriented, boolean isOpenLoop)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.isOpenLoop = isOpenLoop;
    this.isFieldOriented = isFieldOriented;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    swerve.drive(new Translation2d(
            MathUtil.applyDeadband(vX.getAsDouble(), 0.04) * swerve.getSwerveController().config.maxSpeed,
            MathUtil.applyDeadband(vY.getAsDouble(), 0.04)* swerve.getSwerveController().config.maxSpeed),
            MathUtil.applyDeadband(heading.getAsDouble(), 0.04) * swerve.getSwerveController().config.maxAngularVelocity,
            isFieldOriented.getAsBoolean(), isOpenLoop);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}