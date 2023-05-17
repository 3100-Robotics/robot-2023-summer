package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.DoubleSupplier;

public class fieldCentricDrive extends CommandBase {
    private final swerveSubsystem drive;
    private final DoubleSupplier vX, vY, heading;
    private final Boolean isOpenLoop;

    public fieldCentricDrive(swerveSubsystem drive, DoubleSupplier vX, DoubleSupplier vY,
                             DoubleSupplier heading, boolean isOpenLoop) {
        this.drive = drive;
        this.vX = vX;
        this.vY =vY;
        this.heading = heading;
        this.isOpenLoop = isOpenLoop;

        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        ChassisSpeeds wantedSpeeds = drive.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                new Rotation2d(heading.getAsDouble() * Math.PI));

        Translation2d translation = SwerveController.getTranslation2d(wantedSpeeds);
        translation = SwerveMath.limitVelocity(translation, drive.getFieldVelocity(), drive.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                drive.getSwerveDriveConfiguration());

        drive.drive(translation, wantedSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
