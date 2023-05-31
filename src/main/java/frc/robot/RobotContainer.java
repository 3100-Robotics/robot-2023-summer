// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.balance;
import frc.robot.commands.fieldCentricDrive;
import frc.robot.subsystems.*;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

//  define the two cameras
  private final visionWrapper frontCamera = new visionWrapper(
          "frontCamera",
          Constants.visionConstants.robotToFrontCam);
  private final visionWrapper backCamera = new visionWrapper(
          "backCamera",
          Constants.visionConstants.robotToBackCam);

  // define the subsystems
  public final Drive drive = new Drive(frontCamera, backCamera);
  public final Shooter shooter = new Shooter(frontCamera, backCamera);
  public final AngleController angleController = new AngleController(frontCamera, backCamera);
  public final LEDs leds = new LEDs();

  // create the controllers
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController coDriveController = new CommandXboxController(1);

  // create the autonomous selector
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // commented out till I need camera streams on the dashboard
//    CameraServer.startAutomaticCapture();

    // set the default commands of the subsystems
    drive.setDefaultCommand(new fieldCentricDrive(drive, driveController::getLeftX, driveController::getLeftY,
            driveController::getRightX, () -> driveController.rightBumper().getAsBoolean(),false));

    angleController.setDefaultCommand(angleController.runWithJoysticks(coDriveController::getLeftX));

    shooter.setDefaultCommand(shooter.setShooterWithSpeed(-0.02));

    leds.setDefaultCommand(leds.runOnce(leds::showCubeCounter));

    // configure the autonomous routines
    configureAutonomous();

    // Configure the button bindings
    configureBindings();
  }

  /**
   * configure the autonomous routines
   */
  private void configureAutonomous() {
    // create a map of all events that happen in auto
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("shootMid", shooter.runShooterWithVision(Constants.cuberConstants.angles.mid));
    eventMap.put("shootHigh", shooter.runShooterWithVision(Constants.cuberConstants.angles.high));
    eventMap.put("collect",
            angleController.turnToAngle(0).
            andThen(shooter.runShooterSpeedForTime(-0.5, 1).
                    andThen(angleController.turnToAngle(120))));
    eventMap.put("eject", shooter.runShooterSpeedForTime(0.5, 0.5));
    eventMap.put("balance", new balance(drive));

    // create the auto builder
    drive.defineAutoBuilder(
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0), true);

    // create the auto commands
    Command threePieceBalanceLeft = drive.createTrajectory(
            "3 piece balance clean",
            new PathConstraints(8, 4));

    Command threePieceLeft = drive.createTrajectory(
            "3 piece clean",
            new PathConstraints(8, 4));

    Command threePieceBalanceRight = drive.createTrajectory(
            "3 piece balance bump",
            new PathConstraints(8, 4));

    Command threePieceRight = drive.createTrajectory(
            "3 piece bump",
            new PathConstraints(8, 4));

    Command leftTwoPieceCharge = drive.createTrajectory(
            "left 2 piece charge",
            new PathConstraints(8, 4));

    Command rightTwoPieceCharge = drive.createTrajectory(
            "right 2 piece charge",
            new PathConstraints(8, 4));

    Command onePieceCharge = drive.createTrajectory(
            "1 piece charge",
            new PathConstraints(8, 4));

    // add them to the auto chooser
    chooser.setDefaultOption("3 piece balance clean", threePieceBalanceLeft);
    chooser.addOption("3 piece clean", threePieceLeft);
    chooser.setDefaultOption("3 piece balance bump", threePieceBalanceRight);
    chooser.addOption("3 piece bump", threePieceRight);
    chooser.addOption("left 2 piece balance", leftTwoPieceCharge);
    chooser.addOption("right 2 piece balance", rightTwoPieceCharge);
    chooser.addOption("1 piece balance", onePieceCharge);

    // put the chooser to the dashboard
    SmartDashboard.putData("autos", chooser);
  }

  /**
   * configures the button bindings
   */
  private void configureBindings() {
    // set the angles
    coDriveController.a().onTrue(Commands.runOnce(() -> angleController.setAngleSetpoint(0.0)));
    coDriveController.b().onTrue(Commands.runOnce(() -> angleController.setAngleSetpoint(120)));
    // sequence setting the LED color, get and set the angle, set the LED color again,
    // run the shooter, and add another cube to the counter
    coDriveController.x().onTrue(Commands.sequence(
            leds.setColorRGBCommand(255, 204, 0),
            angleController.turnToAngleVision(Constants.cuberConstants.angles.mid),
            leds.showColorTime(51, 204, 51, 2),
            shooter.runShooterWithVision(Constants.cuberConstants.angles.mid),
            leds.incrementCubeCounter()));
    coDriveController.y().onTrue(Commands.sequence(
            leds.showColorTime(255, 204, 0, 1),
            angleController.turnToAngleVision(Constants.cuberConstants.angles.high),
            leds.showColorTime(51, 204, 51, 2),
            shooter.runShooterWithVision(Constants.cuberConstants.angles.high),
            leds.incrementCubeCounter()));

    // collect and shoot
    coDriveController.leftBumper().whileTrue(shooter.setShooterWithSpeed(-0.3));
    coDriveController.rightBumper().whileTrue(shooter.setShooterWithSpeed(0.3).andThen(leds::incrementCubeCounter));
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return the command selected by the auto chooser
    return chooser.getSelected();
  }
}
