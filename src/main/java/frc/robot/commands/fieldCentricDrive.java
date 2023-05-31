package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class fieldCentricDrive extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier vX, vY, heading;

    private final BooleanSupplier isSlowMode;

    private final Boolean isOpenLoop;

    /**
     * a command to drive the robot with joysticks in relation to the field
     * @param drive the drive subsystem
     * @param vX the x velocity supplier
     * @param vY the y velocity supplier
     * @param heading the rotation supplier
     * @param isSlowMode the slow mode button supplier
     * @param isOpenLoop weather or not it is open loop controlled
     */
    public fieldCentricDrive(Drive drive, DoubleSupplier vX, DoubleSupplier vY,
                             DoubleSupplier heading, BooleanSupplier isSlowMode, boolean isOpenLoop) {
        this.drive = drive;
        this.vX = vX;
        this.vY =vY;
        this.heading = heading;
        this.isSlowMode = isSlowMode;
        this.isOpenLoop = isOpenLoop;

        // this command requires the drive subsystem
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // get the target speeds given the inputs and whether the robot is in slow mode
        ChassisSpeeds desiredSpeeds = drive.getTargetSpeeds(
                vX.getAsDouble()*(isSlowMode.getAsBoolean() ? 1 : 0.75),
                vY.getAsDouble()*(isSlowMode.getAsBoolean() ? -1 : -0.75),
                new Rotation2d(heading.getAsDouble() * Math.PI));

        // set the rotation speed to the heading input
        desiredSpeeds.omegaRadiansPerSecond = heading.getAsDouble() * Math.PI;

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, drive.getFieldVelocity(), drive.getPose(),
                Constants.driveConstants.LOOP_TIME, Constants.driveConstants.ROBOT_MASS,
                List.of(Constants.driveConstants.CHASSIS),
                drive.getSwerveDriveConfiguration());

        // dashboard debugging values
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        drive.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
