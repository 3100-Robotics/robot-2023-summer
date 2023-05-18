package frc.robot.subsystems;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.cuberConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;


    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;

    private final BangBangController shooterController;

    private final PhotonCamera frontCamera, backCamera;

    public Shooter(PhotonCamera frontCamera, PhotonCamera backCamera) {
        leftShooter = new CANSparkMax(cuberConstants.leftShooterPort, MotorType.kBrushless);
        rightShooter = new CANSparkMax(cuberConstants.rightShooterPort, MotorType.kBrushless);

        rightShooter.follow(leftShooter);
        leftShooter.setIdleMode(IdleMode.kBrake);
        rightShooter.setIdleMode(IdleMode.kBrake);
        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        leftShooterEncoder = leftShooter.getEncoder();
        rightShooterEncoder = rightShooter.getEncoder();

        shooterController = new BangBangController();
        shooterController.setTolerance(0.5);

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
    }

    // ACTIONS

    public void stopShooter() {
        leftShooter.stopMotor();
    }

    public void runShooterToSpeed() {
        final double vel = leftShooterEncoder.getVelocity();
        final double speed = shooterController.calculate(vel);
        set(speed);
    }

    public void setTargetShooterSpeed(double setpoint) {
        shooterController.setSetpoint(setpoint);
    }


    public void set(double speed) {
        leftShooter.set(speed);
    }

    // GETTERS

    public double getShooterPosition() {
        return (leftShooterEncoder.getPosition() + rightShooterEncoder.getPosition()) / 2;
    }

    // STATES

    public boolean atShooterSpeed() {
        return shooterController.atSetpoint();
    }

    // COMMANDS

    public Command stopShooterCommand() {
        return this.runOnce(this::stopShooter);
    }


    public Command setShooterWithSpeed(double speed) {
        return this.run(() -> set(speed));
    }

    public Command runShooterSpeedForTime(double speed, double time) {
        double startTime = Timer.getFPGATimestamp();
        return this.runOnce(() -> setTargetShooterSpeed(speed)).
                andThen(this::runShooterToSpeed).until(() -> Timer.getFPGATimestamp() - startTime >= time);
    }

    public Command runShooterWithVision(String level) {
        PhotonPipelineResult frontResults = frontCamera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double angle;

        double distance = 0;

        int numLevel;

        if (level.equals("mid")) {
            numLevel = 0;
        }
        else {
            numLevel = 1;
        }

        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        angle = Math.atan(
                (2/distance) *
                        (Constants.visionConstants.heightDiffs[numLevel] + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                        Constants.visionConstants.heightDiffs[numLevel] *
                                                Constants.visionConstants.maxHeight)));

        return runShooterSpeedForTime((Math.sqrt(2* Constants.visionConstants.g*
                (Constants.visionConstants.heightDiffs[numLevel] +
                        Constants.visionConstants.maxHeight)))/Math.sin(angle), 0.5);
    }
}

