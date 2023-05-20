package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.cuberConstants;
import frc.robot.visionWrapper;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AngleController extends SubsystemBase {
    private final CANSparkMax angleMotor;

    private final AbsoluteEncoder angleEncoder;

    private final PIDController angleController;

    private final visionWrapper frontCamera, backCamera;
    public AngleController(visionWrapper frontCamera, visionWrapper backCamera) {
        angleMotor = new CANSparkMax(cuberConstants.angleMotorPort, MotorType.kBrushless);

        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setInverted(false);

        angleEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        angleController = new PIDController(
                cuberConstants.angleP,
                cuberConstants.angleI,
                cuberConstants.angleD);
        angleController.setTolerance(0.1);

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
    }

    // ACTIONS

    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    public void moveToTargetAngle() {
        final double angle = angleEncoder.getPosition();
        final double speed = angleController.calculate(angle);
        setAngleMotor(speed);
    }

    public void setTargetAngle(double setpointStatic) {
        angleController.reset();
        angleController.setSetpoint(setpointStatic);
    }

    public void setTargetAngleVision(cuberConstants.angles level) {
        PhotonPipelineResult frontResults = frontCamera.camera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.camera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double distance = 0;

        int numLevel = 1;

        if (level.equals(cuberConstants.angles.mid)) {
            numLevel = 0;
        }

        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        setTargetAngle(Math.atan(
                (2/distance) *
                        (Constants.visionConstants.heightDiffs[numLevel] + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                        Constants.visionConstants.heightDiffs[numLevel] *
                                                Constants.visionConstants.maxHeight))));
    }

    public void setAngleMotor(double speed) {
        angleMotor.set(speed);
    }

    // GETTERS

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    // STATES

    public boolean angleStopped() {
        return angleEncoder.getVelocity() < 0.01;
    }

    public boolean atAngle() {
        return angleController.atSetpoint();
    }

    // COMMANDS

    public Command stopAngleMotorCommand() {
        return this.runOnce(this::stopAngleMotor);
    }

    public Command setAngleWithSpeed(double speed) {
        return this.run(() -> setAngleMotor(speed));
    }

    public Command updateAngle() {
        return this.run(this::moveToTargetAngle);
    }

    public Command turnToAngle(double angle) {
        return this.
                runOnce(() -> setTargetAngle(angle)).
                andThen(run(this::moveToTargetAngle)).until(this::atAngle);
    }

    public Command turnToAngleVision(cuberConstants.angles level) {
        return this.
                runOnce(() -> setTargetAngleVision(level)).
                andThen(run(this::moveToTargetAngle)).until(this::atAngle);
    }
}

