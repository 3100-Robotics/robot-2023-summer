// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.balance;
import frc.robot.commands.fieldCentricDrive;
import frc.robot.subsystems.*;
import org.photonvision.PhotonCamera;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final PhotonCamera frontCamera = new PhotonCamera("frontCamera");
  private final PhotonCamera backCamera = new PhotonCamera("backCamera");

  public final swerveSubsystem drive = new swerveSubsystem();
  public final Shooter shooter = new Shooter(frontCamera, backCamera);
  public final AngleController angleController = new AngleController(frontCamera, backCamera);
  public final LEDs leds = new LEDs();

  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController coDriveController = new CommandXboxController(1);

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new fieldCentricDrive(drive, driveController::getLeftX, driveController::getLeftY,
            driveController::getRightX, () -> driveController.rightBumper().getAsBoolean(),false));

    angleController.setDefaultCommand(angleController.updateAngle());

    shooter.setDefaultCommand(shooter.setShooterWithSpeed(-0.02));

    configureAutonomous();

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureAutonomous() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("shootMid", shooter.runShooterWithVision("mid"));
    eventMap.put("shootHigh", shooter.runShooterWithVision("high"));
    eventMap.put("collect", angleController.turnToAngle(0).
            andThen(shooter.runShooterSpeedForTime(-0.5, 1).
                    andThen(angleController.turnToAngle(120))));
    eventMap.put("eject", shooter.runShooterSpeedForTime(0.5, 0.5));
    eventMap.put("balance", new balance(drive));

    Command threePieceBalanceLeft = drive.createTrajectory(
            "3 piece balance clean",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    Command threePieceLeft = drive.createTrajectory(
            "3 piece clean",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    Command threePieceBalanceRight = drive.createTrajectory(
            "3 piece balance bump",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    Command threePieceRight = drive.createTrajectory(
            "3 piece bump",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    Command leftTwoPieceCharge = drive.createTrajectory(
            "left 2 piece charge",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    Command rightTwoPieceCharge = drive.createTrajectory(
            "right 2 piece charge",
            new PathConstraints(8, 4),
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            true);

    chooser.setDefaultOption("3 piece balance clean", threePieceBalanceLeft);
    chooser.addOption("3 piece clean", threePieceLeft);
    chooser.setDefaultOption("3 piece balance bump", threePieceBalanceRight);
    chooser.addOption("3 piece bump", threePieceRight);
    chooser.addOption("left 2 piece balance", leftTwoPieceCharge);
    chooser.addOption("right 2 piece balance", rightTwoPieceCharge);

    SmartDashboard.putData("autos", chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    coDriveController.a().onTrue(new InstantCommand(() -> angleController.setTargetAngle(0.0)));
    coDriveController.b().onTrue(new InstantCommand(() -> angleController.setTargetAngle(120)));
    coDriveController.x().onTrue(angleController.runOnce(() -> leds.setColorRGB(255, 204, 0)).
            andThen(angleController.turnToAngleVision("mid")).
            andThen(() -> leds.setColorRGB(51, 204, 51)).
            andThen(shooter.runShooterWithVision("mid")));
    coDriveController.x().onTrue(angleController.runOnce(() -> leds.setColorRGB(255, 204, 0)).
            andThen(angleController.turnToAngleVision("high")).
            andThen(() -> leds.setColorRGB(51, 204, 51)).
            andThen(shooter.runShooterWithVision("high")));

    coDriveController.leftBumper().whileTrue(shooter.setShooterWithSpeed(-0.3));
    coDriveController.rightBumper().whileTrue(shooter.setShooterWithSpeed(0.3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
