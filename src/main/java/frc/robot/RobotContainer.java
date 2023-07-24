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
import frc.robot.commands.drive;
import frc.robot.commands.fieldCentricDrive;
import frc.robot.subsystems.*;
import frc.robot.Constants.visionConstants.cameraType;
import frc.robot.vision.visionWrapper;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based
 * is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and trigger mappings)
 * should be declared here.
 */
public class RobotContainer {

  // let me enable and disable certain subsystems for testing
  public final Boolean shooterEnabled = false;
  public final Boolean angleControllerEnabled = false;
  public final Boolean driveEnabled = true;
  public final Boolean LEDsEnabled = false;


  // create the subsystem vars
  public Drive drive;
  public Shooter shooter;
  public AngleController angleController;
  public LEDs leds;


  // create the controllers
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController coDriveController = new CommandXboxController(1);


  // create the autonomous selector
  private final SendableChooser<Command> chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //  define the three cameras
    visionWrapper frontCamera = new visionWrapper(
            "frontCamera",
            Constants.visionConstants.robotToFrontCam, cameraType.photonVision);
    visionWrapper backCamera = new visionWrapper(
            "backCamera",
            Constants.visionConstants.robotToBackCam, cameraType.photonVision);
    visionWrapper gpCamera = new visionWrapper("limelight",
            Constants.visionConstants.robotToLimelight, cameraType.LimeLight);

    // define and set the default command of the subsystems if they are enabled
    if (driveEnabled) {
      drive = new Drive(frontCamera, backCamera);
      drive.setDefaultCommand(new drive(drive, driveController::getLeftX,
              driveController::getLeftY, driveController::getRightX,
              () -> SmartDashboard.getBoolean("is field oriented", true),false, true));
    }
    if (shooterEnabled) {
      shooter = new Shooter(frontCamera, backCamera);
      shooter.setDefaultCommand(shooter.setShooterWithSpeed(-0.02));
    }
    if (angleControllerEnabled) {
      angleController = new AngleController(frontCamera, backCamera);
      angleController.setDefaultCommand(angleController.runWithJoysticks(coDriveController::getLeftX));
    }
    if (LEDsEnabled) {
      leds = new LEDs();
      leds.setDefaultCommand(leds.runOnce(leds::showCubeCounter));
    }

    // commented out till I need camera streams on the dashboard
//    CameraServer.startAutomaticCapture();


    // configure the autonomous routines
    configureAutonomous();

    // Configure the button bindings
    configureBindings();
  }

  /**
   * configure the autonomous routines
   */
  private void configureAutonomous() {
    // this will only run if the required subsystems are enabled
    if (driveEnabled && shooterEnabled && angleControllerEnabled) {
      // create a map of all events that can happen in auto
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("shootMid", shooter.runShooterWithVision(Constants.visionConstants.heights.mid));
      eventMap.put("shootHigh", shooter.runShooterWithVision(Constants.visionConstants.heights.high));
      eventMap.put("shootLow", shooter.runShooterWithVision(Constants.visionConstants.heights.low));
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

      // create constraints for velocity and acceleration
      PathConstraints constraints = new PathConstraints(6, 4);

      // create the auto commands
      // autos for the left side of the community
      Command threePieceBalanceLeft = drive.createTrajectory(
              "3 piece balance clean",
              constraints);

      Command threePieceLeft = drive.createTrajectory(
              "3 piece clean",
              constraints);

      Command fourPieceLeft = drive.createTrajectory(
              "4 piece clean", constraints);

      Command fivePieceLeft = drive.createTrajectory(
              "5 piece clean", constraints);

      // create autos for the right side of the community

      Command threePieceBalanceRight = drive.createTrajectory(
              "3 piece balance bump",
              constraints);

      Command threePieceRight = drive.createTrajectory(
              "3 piece bump",
              constraints);

      Command fourPieceRight = drive.createTrajectory(
              "4 piece bump",
              constraints);

      Command fivePieceRight = drive.createTrajectory(
              "5 piece bump",
              constraints);

      // create autos for the middle of the community

      Command leftTwoPieceCharge = drive.createTrajectory(
              "left 2 piece charge",
              constraints);

      Command rightTwoPieceCharge = drive.createTrajectory(
              "right 2 piece charge",
              constraints);

      Command onePieceCharge = drive.createTrajectory(
              "1 piece charge",
              constraints);

      Command threePieceCharge = drive.createTrajectory(
              "3 piece balance mid",
              constraints);

      // add them to the auto chooser
      // left side autos
      chooser.setDefaultOption("3 piece balance clean", threePieceBalanceLeft);
      chooser.addOption("3 piece clean", threePieceLeft);
      chooser.addOption("4 piece clean", fourPieceLeft);
      chooser.addOption("5 piece clean", fivePieceLeft);
      // right side autos
      chooser.addOption("3 piece balance bump", threePieceBalanceRight);
      chooser.addOption("3 piece bump", threePieceRight);
      chooser.addOption("4 piece bump", fourPieceRight);
      chooser.addOption("5 piece bump", fivePieceRight);
      // middle autos
      chooser.addOption("3 piece balance middle", threePieceCharge);
      chooser.addOption("left 2 piece balance middle", leftTwoPieceCharge);
      chooser.addOption("right 2 piece balance middle", rightTwoPieceCharge);
      chooser.addOption("1 piece balance middle", onePieceCharge);
    }
    // add a do nothing auto
    chooser.addOption("do nothing", Commands.none());

    // put the chooser to the dashboard
    SmartDashboard.putData("autos", chooser);
  }

  /**
   * configures the button bindings
   */
  private void configureBindings() {
    // all ifs are checks to see if the required subsystems are enabled
    // basic angles (collection angle and a stowed angle)
    if (angleControllerEnabled) {
      coDriveController.a().onTrue(Commands.runOnce(() -> angleController.setAngleSetpoint(0.0)));
      coDriveController.b().onTrue(Commands.runOnce(() -> angleController.setAngleSetpoint(120)));
    }

    // dynamic angles. These are to aim for high and mid from any distance within the community
    if (driveEnabled && angleControllerEnabled && shooterEnabled && LEDsEnabled) {
      // sequence locking the drivetrain, setting the LED color, get and set the angle, shoot,
      // update the LED color, and up the number of cubes shot
      coDriveController.x().onTrue(Commands.sequence(
              Commands.runOnce(drive::lock),
              leds.setColorRGBCommand(255, 204, 0),
              angleController.turnToAngleVision(Constants.visionConstants.heights.mid),
              shooter.runShooterWithVision(Constants.visionConstants.heights.mid),
              leds.showColorTime(51, 204, 51, 2),
              leds.incrementCubeCounter()));
      coDriveController.y().onTrue(Commands.sequence(
              Commands.runOnce(drive::lock),
              leds.setColorRGBCommand(255, 204, 0),
              angleController.turnToAngleVision(Constants.visionConstants.heights.high),
              shooter.runShooterWithVision(Constants.visionConstants.heights.high),
              leds.showColorTime(51, 204, 51, 2),
              leds.incrementCubeCounter()));
    }
    // allow the button bindings to be created without the LEDs
    else if (driveEnabled && angleControllerEnabled && shooterEnabled) {
      coDriveController.x().onTrue(Commands.sequence(
              Commands.runOnce(drive::lock),
              angleController.turnToAngleVision(Constants.visionConstants.heights.mid),
              shooter.runShooterWithVision(Constants.visionConstants.heights.mid)));
      coDriveController.y().onTrue(Commands.sequence(
              Commands.runOnce(drive::lock),
              angleController.turnToAngleVision(Constants.visionConstants.heights.high),
              shooter.runShooterWithVision(Constants.visionConstants.heights.high)));
    }

    // collect and shoot. These are for running the wheels to collect or shoot
    if (shooterEnabled && LEDsEnabled) {
      coDriveController.leftBumper().onTrue(
              shooter.collect(-0.3)
                      .andThen(() -> leds.setColorRGBCommand(0, 230, 0)));
      coDriveController.rightBumper().whileTrue(
              shooter.setShooterWithSpeed(0.3).andThen(leds::incrementCubeCounter));
    }
    // allow the button bindings to be created without the LEDs
    else if (shooterEnabled) {
      coDriveController.leftBumper().onTrue(
              shooter.collect(-0.3));
      coDriveController.rightBumper().whileTrue(
              shooter.setShooterWithSpeed(0.3));
    }
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return the command selected by the auto chooser
    return chooser.getSelected();
  }
}
