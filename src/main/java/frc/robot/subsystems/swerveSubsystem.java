package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.visionWrapper;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import swervelib.telemetry.SwerveDriveTelemetry;

public class swerveSubsystem extends SubsystemBase {
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive drive;

   private SwerveAutoBuilder autoBuilder = null;

   private final PIDController balanceController;

   private final visionWrapper frontCamera, backCamera;

    private final Field2d field = new Field2d();

    public swerveSubsystem(visionWrapper frontCamera, visionWrapper backCamera) {

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;

        balanceController  = new PIDController(
                driveConstants.balanceP,
                driveConstants.balanceI,
                driveConstants.balanceD);
        balanceController.setTolerance(0.3, 1);
        balanceController.setSetpoint(0);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        SmartDashboard.putData("Field", field);

        try {
            drive = new SwerveParser(swerveJsonDir).createSwerveDrive();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(drive.swerveDrivePoseEstimator.getEstimatedPosition());
        SmartDashboard.putNumberArray("2d pos", new double[]{
                drive.swerveDrivePoseEstimator.getEstimatedPosition().getX(),
                drive.swerveDrivePoseEstimator.getEstimatedPosition().getY(),
                drive.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians()});
    }

    @Override
    public void simulationPeriodic() {}


    public void updateOdometry() {
        drive.updateOdometry();

        Optional<EstimatedRobotPose> frontResult = frontCamera.getEstimatedGlobalPose(drive.getPose());
        Optional<EstimatedRobotPose> backResult = frontCamera.getEstimatedGlobalPose(drive.getPose());

        if (frontResult.isPresent()) {
            EstimatedRobotPose camPose = frontResult.get();
            drive.swerveDrivePoseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
        if (backResult.isPresent()) {
            EstimatedRobotPose camPose = backResult.get();
            drive.swerveDrivePoseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public SwerveKinematics2 getKinematics() {
        return drive.kinematics;
    }


    public Pose2d getPose() {
        return drive.getPose();
    }


    public void resetOdometry(Pose2d initialHolonomicPose) {
        drive.resetOdometry(initialHolonomicPose);
    }


    public Rotation2d getHeading()
    {
        return drive.getYaw();
    }


    public Rotation2d getPitch() {
        return drive.getPitch();
    }


    public Rotation2d getRoll() {
        return drive.getRoll();
    }


    public double calculate(double measurement) {
        return balanceController.calculate(measurement);
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


    public void lock() {
        drive.lockPose();
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
