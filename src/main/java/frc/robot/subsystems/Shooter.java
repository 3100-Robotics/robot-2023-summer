package frc.robot.subsystems;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.cuberConstants;
import frc.robot.visionWrapper;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * this class allows you to control the shooter either manually or with vision.
 */
public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;

    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;

    private final BangBangController shooterController;

    private final visionWrapper frontCamera, backCamera;

    /**
     * constructs a new shooter that has access to the given cameras
     * @param frontCamera the camera in front
     * @param backCamera the camera in back
     */
    public Shooter(visionWrapper frontCamera, visionWrapper backCamera) {
        leftShooter = new CANSparkMax(cuberConstants.leftShooterPort, MotorType.kBrushless);
        rightShooter = new CANSparkMax(cuberConstants.rightShooterPort, MotorType.kBrushless);

        // motor configuration
        rightShooter.follow(leftShooter);
        leftShooter.setIdleMode(IdleMode.kBrake);
        rightShooter.setIdleMode(IdleMode.kBrake);
        leftShooter.setInverted(false);
        rightShooter.setInverted(false);
        leftShooter.setSmartCurrentLimit(50);
        rightShooter.setSmartCurrentLimit(50);

        // set the encoders to be the motor's encoders
        leftShooterEncoder = leftShooter.getEncoder();
        rightShooterEncoder = rightShooter.getEncoder();

        shooterController = new BangBangController();
        shooterController.setTolerance(0.5);

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
    }

    @Override
    public void periodic() {
        // dashboard debugging values
        SmartDashboard.putNumberArray("SmartDashboard/shooter shooter speeds", new double[]{
                leftShooterEncoder.getVelocity(),
                rightShooterEncoder.getVelocity()});
    }

    // ACTIONS

    public void stopShooter() {
        leftShooter.stopMotor();
    }

    /**
     * set the shooter with the velocity given from the bang bang controller
     */
    public void runShooterToSpeed() {
        final double vel = leftShooterEncoder.getVelocity();
        final double speed = shooterController.calculate(vel);
        set(speed);
    }

    public void setShooterSpeedSetpoint(double setpoint) {
        shooterController.setSetpoint(setpoint);
    }


    /**
     * @param speed the wanted speed of the motor
     */
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

    /**
     * create a command to run the shooter at a speed for a given time
     * @param speed the speed to run
     * @param time the time to run that speed for
     * @return the generated command
     */
    public Command runShooterSpeedForTime(double speed, double time) {
        return this.runOnce(() -> setShooterSpeedSetpoint(speed)).
                andThen(this::runShooterToSpeed).deadlineWith(Commands.waitSeconds(time));
    }

    /**
     * create a command to run the shooter at a certain speed given the angle to aim for
     * @param level the shelf level to aim for
     * @return the generated command
     */
    public Command runShooterWithVision(cuberConstants.angles level) {
        // get the camera results
        PhotonPipelineResult frontResults = frontCamera.camera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.camera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double angle;

        double distance = 0;

        int numLevel;

        if (level.equals(cuberConstants.angles.low)) {
            numLevel = 0;
        }
        else if (level.equals(cuberConstants.angles.mid)){
            numLevel = 1;
        }
        else {
            numLevel = 2;
        }

        // check if either of the cameras have targets.
        // if they do get their best targets. defaults to the front camera
        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        // calculate the speed to run at
        angle = Math.atan(
                (2/distance) *
                        (Constants.visionConstants.heightDiffs[numLevel] + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                        Constants.visionConstants.heightDiffs[numLevel] *
                                                Constants.visionConstants.maxHeight)));

        // create and return the command
        return runShooterSpeedForTime((Math.sqrt(2* Constants.visionConstants.g*
                (Constants.visionConstants.heightDiffs[numLevel] +
                        Constants.visionConstants.maxHeight)))/Math.sin(angle), 1);
    }
}

