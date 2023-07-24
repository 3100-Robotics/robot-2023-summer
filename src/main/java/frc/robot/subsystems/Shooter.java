package frc.robot.subsystems;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.cuberConstants;
import frc.robot.vision.results;
import frc.robot.vision.visionWrapper;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * this class allows you to control the shooter either manually or with vision.
 */
public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;

    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;

    private final SparkMaxPIDController leftShooterController;

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
        configureMotors();

        // set the encoders to be the motor's encoders
        leftShooterEncoder = leftShooter.getEncoder();
        rightShooterEncoder = rightShooter.getEncoder();

        // set up the pid controller
        leftShooterController = leftShooter.getPIDController();
        leftShooterController.setP(cuberConstants.shooterP);
        leftShooterController.setI(cuberConstants.shooterI);
        leftShooterController.setD(cuberConstants.shooterD);
        leftShooterController.setFeedbackDevice(leftShooterEncoder);

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;
    }

    /**
     * configure the motors.
     */
    private void configureMotors() {
        rightShooter.follow(leftShooter);
        leftShooter.setIdleMode(IdleMode.kBrake);
        rightShooter.setIdleMode(IdleMode.kBrake);
        leftShooter.setInverted(false);
        rightShooter.setInverted(false);
        leftShooter.setSmartCurrentLimit(50);
        rightShooter.setSmartCurrentLimit(50);
    }

    @Override
    public void periodic() {
        // dashboard debugging values
        SmartDashboard.putNumberArray("SmartDashboard/shooter shooter speeds", new double[]{
                leftShooterEncoder.getVelocity(),
                rightShooterEncoder.getVelocity()});
        SmartDashboard.putNumber("SmartDashboard/shooter shooter current",
                (leftShooter.getOutputCurrent()+rightShooter.getOutputCurrent())/2);
    }

    // ACTIONS

    public void stopShooter() {
        leftShooter.stopMotor();
    }

    public void setShooterSpeedSetpoint(double setpoint) {
        leftShooterController.setReference(
                setpoint/ cuberConstants.shooterGearRatio,
                CANSparkMax.ControlType.kVelocity);
    }


    /**
     * @param speed the wanted speed of the motor
     */
    public void set(double speed) {
        leftShooter.set(speed);
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
        return this.runOnce(() -> setShooterSpeedSetpoint(speed)).deadlineWith(Commands.waitSeconds(time));
    }

    /**
     * a command to run the motors until they detect that they have collected a cube
     * @param speed the speed to run at
     * @return the generated command
     */
    public Command collect(double speed) {
        return this.run(() -> set(speed)).until(() -> leftShooter.getOutputCurrent()>30&&
                rightShooter.getOutputCurrent()>30).andThen(this::stopShooter);
    }

    /**
     * create a command to run the shooter at a certain speed given the angle to aim for
     * @param level the shelf level to aim for
     * @return the generated command
     */
    public Command runShooterWithVision(Constants.visionConstants.heights level) {
        // get the camera results
        results frontResults = frontCamera.getLatestResult();
        results backResults = backCamera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double angle;

        double distance = 0;

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
                        (level.getHeightDiff() + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                        level.getHeightDiff() *
                                                Constants.visionConstants.maxHeight)))+(Math.PI/2);

        // create and return the command
        return runShooterSpeedForTime((Math.sqrt(2* Constants.visionConstants.g*
                (level.getHeightDiff() +
                        Constants.visionConstants.maxHeight)))/Math.sin(angle)
                        /(cuberConstants.shooterWheelRadius*Math.PI),
                1);
    }
}

