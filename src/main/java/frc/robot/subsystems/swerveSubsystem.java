package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class swerveSubsystem extends SubsystemBase {
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive drive;

   private SwerveAutoBuilder autoBuilder = null;

    public swerveSubsystem() {
        try {
            drive = new SwerveParser(swerveJsonDir).createSwerveDrive();
        } catch (IOException ignored) {}
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public SwerveKinematics2 getKinematics() {
        return drive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        drive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public Rotation2d getHeading()
    {
        return drive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return drive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return drive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity() {
        return drive.getFieldVelocity();
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
        return drive.swerveDriveConfiguration;
    }

    public SwerveController getSwerveController() {
        return drive.swerveController;
    }

    public Command createTrajectory(String path, PathConstraints constraints, Map<String, Command> eventMap,
                                    PIDConstants translation, PIDConstants rotation, boolean useAllianceColor) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);
        if (autoBuilder == null) {
            autoBuilder = new SwerveAutoBuilder(
                    drive::getPose,
                    drive::resetOdometry,
                    translation,
                    rotation,
                    drive::setChassisSpeeds,
                    eventMap,
                    useAllianceColor,
                    this);
        }

        return autoBuilder.fullAuto(pathGroup);
    }
}
