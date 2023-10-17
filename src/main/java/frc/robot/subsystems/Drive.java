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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.vision.visionWrapper;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * This class sets up everything needed for the drivetrain.
 * This includes, the drivetrain from the library, the autonomous builder,
 * the pose estimator, and the debugging values for the dashboard
 */
public class Drive extends SubsystemBase {

	// define the directory that contains the config files for the swerve drive
	File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
	SwerveDrive drive;

    private SwerveAutoBuilder autoBuilder = null;

    // PID controller for balancing
    private final PIDController balanceController;

    private final visionWrapper frontCamera, backCamera;

    // 2d field to put to the dashboard
	private final Field2d field = new Field2d();

    /**
     * constructs a new drivetrain.
     * @param frontCamera the camera facing the front
     * @param backCamera the camera facing the back
     */
	public Drive(visionWrapper frontCamera, visionWrapper backCamera) {

		this.frontCamera = frontCamera;
		this.backCamera = backCamera;

        // set the balance controller's P, I, D, tolerance, and setpoint
		balanceController  = new PIDController(
				driveConstants.balanceP,
				driveConstants.balanceI,
				driveConstants.balanceD);
		balanceController.setTolerance(0.3, 1);
		balanceController.setSetpoint(0);

		// put the 2d field to the dashboard
		SmartDashboard.putData("Field", field);

		// set the swerve telemetry's verbosity
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        // create the drivetrain from the config files
		try {
			drive = new SwerveParser(swerveJsonDir).createSwerveDrive();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}


	@Override
	public void periodic() {
        // update the robot pose
		updateOdometry();

        // give the 2d field the updated pose
		field.setRobotPose(drive.swerveDrivePoseEstimator.getEstimatedPosition());
	}

	public void updateOdometry() {
        // use the encoders to estimate pose.
		drive.updateOdometry();

        // get the estimated poses from the cameras.
		Optional<EstimatedRobotPose> frontResult = frontCamera.getEstimatedGlobalPose(drive.getPose());
		Optional<EstimatedRobotPose> backResult = backCamera.getEstimatedGlobalPose(drive.getPose());

        // if the results exist (if there are april tags in view)
		// give them to the pose estimator.
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


    /**
     * A function to supply the drivetrain with movement commands
     * @param translation the wanted translation.
     * @param rotation the wanted rotation.
     * @param fieldRelative whether the robot is driving relative to the field.
     * @param isOpenLoop whether to drive in open loop mode
     */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		drive.drive(translation, rotation, fieldRelative, isOpenLoop);
	}

	public void setChassisSpeeds(ChassisSpeeds speeds) {
		drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond,
				drive.getYaw()));
	}

    /**
     * a way to get the swerve drive's kinematics
     * @return the swerve drive's kinematics.
     */
	public SwerveDriveKinematics getKinematics() {
		return drive.kinematics;
	}


    /**
     * get the estimated pose of the robot
     * @return the pose of the robot
     */
	public Pose2d getPose() {
		return drive.getPose();
	}


    /**
     * resets the odometry to the given pose.
     * @param initialHolonomicPose the initial pose to reset to
     */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		drive.resetOdometry(initialHolonomicPose);
	}

    /**
     * @return the heading of the robot from the gyro
     */
	public Rotation2d getHeading() {
		return drive.getYaw();
	}

    /**
     * @return the pitch of the robot from the gyro
     */
	public Rotation2d getPitch() {
		return drive.getPitch();
	}

    /**
     * @return the roll of the robot from the gyro
     */
	public Rotation2d getRoll() {
		return drive.getRoll();
	}


    /**
     * calculate the desired velocity of the robot when balancing.
     * @param measurement the current angle of the robot
     * @return the desired velocity
     */
	public double calculate(double measurement) {
		return balanceController.calculate(measurement);
	}


    /**
     * get the target speeds for the robot based off of 4 axis.
     * @param xInput the x-axis input of the first joystick
     * @param yInput the y-axis input of the first joystick
     * @param headingX the x-axis input of the second joystick
     * @param headingY the y-axis input of the second joystick
     * @return the target speeds
     */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        // cube the translation inputs for more controllability
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return drive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
	}


    /**
     * get the target speeds for the robot based off of 3 axis
     * @param xInput the 1st input axis
     * @param yInput the 2nd input axis
     * @param angle the 3rd input axis
     * @return the target speeds
     */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        // cube the translation inputs for more controllability
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return drive.swerveController.getTargetSpeeds(xInput, yInput,
				angle.getRadians(), getHeading().getRadians());
	}


    /**
     * @return get the field relative velocity
     */
	public ChassisSpeeds getFieldVelocity() {
		return drive.getFieldVelocity();
	}


    /**
     * @return the current swerve drive configuration
     */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return drive.swerveDriveConfiguration;
	}

    /**
     * @return get the swerve heading controller
     */
	public SwerveController getSwerveController() {
		return drive.swerveController;
	}


	public Command lineUpWithTag() {
		// get the camera results
		PhotonPipelineResult frontResults = frontCamera.getLatestResult();
		PhotonPipelineResult backResults = backCamera.getLatestResult();

		PhotonTrackedTarget frontBestTarget;
		PhotonTrackedTarget backBestTarget;

		// check if either of the cameras have targets.
		// if they do get their best targets. defaults to the front camera
		if (frontResults.hasTargets()) {
			frontBestTarget = frontResults.getBestTarget();
			return this.run(() -> drive(
				new Translation2d(0, 0), Math.copySign(10, frontBestTarget.getYaw()), false, false));
		}
		else if (backResults.hasTargets()) {
			backBestTarget = backResults.getBestTarget();
			return this.run(() -> drive(
				new Translation2d(0, 0), Math.copySign(10, -backBestTarget.getYaw()), false, false));
		}
		return Commands.none();
	}


	/**
	 * make the swerve drive's wheels go in an x pattern to force
	 * the robot to stay in position
	 */
	public void lock() {
		drive.lockPose();
	}

	/**
	 * define the autonomous builder.
	 * @param eventMap the list of all events that occur in any auto
	 * @param translationPID the pid constants that should be used for translation
	 * @param rotationPID the pid constants that should be used for rotation
	 * @param useAllianceColor whether to account for the
	 *                           alliance color while creating an auto
	 */
	public void defineAutoBuilder(Map<String, Command> eventMap,
								  PIDConstants translationPID,
								  PIDConstants rotationPID, boolean useAllianceColor) {
		if (autoBuilder == null) {
			autoBuilder = new SwerveAutoBuilder(
					drive::getPose,
					drive::resetOdometry,
					translationPID,
					rotationPID,
					this::setChassisSpeeds,
					eventMap,
					useAllianceColor,
					this);
		}
	}

	/**
	 * create an auto from the given path and constrains
	 * @param path the path of the auto
	 * @param constraints the velocity and acceleration constraints
	 * @return the auto
	 */
	public Command createTrajectory(String path, PathConstraints constraints) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);

		return autoBuilder.fullAuto(pathGroup);
	}
}
