package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class fieldCentricDrive extends CommandBase {
    private final swerveSubsystem drive;
    private final DoubleSupplier vX, vY, heading;

    private final BooleanSupplier isSlowMode;

    private final Boolean isOpenLoop;

    public fieldCentricDrive(swerveSubsystem drive, DoubleSupplier vX, DoubleSupplier vY,
                             DoubleSupplier heading, BooleanSupplier isSlowMode, boolean isOpenLoop) {
        this.drive = drive;
        this.vX = vX;
        this.vY =vY;
        this.heading = heading;
        this.isSlowMode = isSlowMode;
        this.isOpenLoop = isOpenLoop;

        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        ChassisSpeeds desiredSpeeds = drive.getTargetSpeeds(
                vX.getAsDouble()*(isSlowMode.getAsBoolean() ? 1 : 0.75),
                vY.getAsDouble()*(isSlowMode.getAsBoolean() ? -1 : -0.75),
                new Rotation2d(heading.getAsDouble() * Math.PI));

        desiredSpeeds.omegaRadiansPerSecond = heading.getAsDouble() * Math.PI;

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, drive.getFieldVelocity(), drive.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                drive.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        drive.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
