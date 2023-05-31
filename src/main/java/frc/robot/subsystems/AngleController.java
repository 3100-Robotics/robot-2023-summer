package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.cuberConstants;
import frc.robot.visionWrapper;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.function.DoubleSupplier;

/**
 * this class allows you to control the angle motor either manually or with vision
 */
public class AngleController extends SubsystemBase {
    private final CANSparkMax angleMotor;

    private final AbsoluteEncoder angleEncoder;

    private final PIDController angleController;

    private final visionWrapper frontCamera, backCamera;

    // testing out simulating the claw
    private final Mechanism2d mech = new Mechanism2d(10, 11);
    private final MechanismRoot2d root = mech.getRoot("root", 1, 0);
    private final MechanismLigament2d claw = root.append(new MechanismLigament2d("claw", 9.8, 0));

    /**
     * constructs a new angle controller and gives it two cameras.
     * @param frontCamera the camera in front
     * @param backCamera the camera in back
     */
    public AngleController(visionWrapper frontCamera, visionWrapper backCamera) {
        angleMotor = new CANSparkMax(cuberConstants.angleMotorPort, MotorType.kBrushless);

        // configure the motor
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setInverted(false);
        angleMotor.setSmartCurrentLimit(50);

        // set the encoder to be the connected absolute encoder
        angleEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // set up the pid controller
        angleController = new PIDController(
                cuberConstants.angleP,
                cuberConstants.angleI,
                cuberConstants.angleD);
        angleController.setTolerance(0.1);

        this.frontCamera = frontCamera;
        this.backCamera = backCamera;

        // put the mechanism to the dashboard
        SmartDashboard.putData("claw", mech);

        // if it's a simulation, add a spark max to it.
        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(angleMotor, DCMotor.getNEO(1));
        }
    }

    @Override
    public void periodic() {
        // update the angle of the simulated claw
        claw.setAngle(angleEncoder.getPosition()*360);
    }

    @Override
    public void simulationPeriodic() {
        // update the simulation and put the speed of the angle motor
        REVPhysicsSim.getInstance().run();
        SmartDashboard.putNumber("angle speed", angleMotor.getEncoder().getVelocity());
    }

    // ACTIONS

    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    /**
     * set the motor to the calculated speed from the pid controller
     */
    public void moveToTargetAngle() {
        final double angle = angleEncoder.getPosition();
        final double speed = angleController.calculate(angle);
        setAngleMotor(speed);
    }

    /**
     * set the setpoint angle
     * @param setpoint the wanted setpoint
     */
    public void setAngleSetpoint(double setpoint) {
        angleController.reset();
        angleController.setSetpoint(setpoint);
    }

    /**
     * set the angle of the shooter according to the wanted shelf to shoot too
     * @param level the shelf level
     */
    public void setTargetAngleVision(cuberConstants.angles level) {
        // get the latest results
        PhotonPipelineResult frontResults = frontCamera.camera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.camera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double distance = 0;

        int numLevel = 1;

        if (level.equals(cuberConstants.angles.mid)) {
            numLevel = 0;
        }

        // if there are results get the distance from them. The front camera is prioritized
        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        // calculate the angle and set it as the setpoint
        setAngleSetpoint(Math.atan(
                (2/distance) *
                        (Constants.visionConstants.heightDiffs[numLevel] + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                        Constants.visionConstants.heightDiffs[numLevel] *
                                                Constants.visionConstants.maxHeight))));
    }

    /**
     * set the speed of the angle motor
     * @param speed the desired speed
     */
    public void setAngleMotor(double speed) {
        angleMotor.set(speed);
    }

    // GETTERS

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    // STATES

    /**
     * @return determines if the angle motor is stopped
     */
    public boolean angleStopped() {
        return angleEncoder.getVelocity() < 0.01;
    }

    public boolean atSetpoint() {
        return angleController.atSetpoint();
    }

    // COMMANDS

    /**
     * create a command to stop the angle motor.
     * @return the generated command
     */
    public Command stopAngleMotorCommand() {
        return this.runOnce(this::stopAngleMotor);
    }

    /**
     * create a command to run the angle motor with a speed
     * @param speed the desired speed
     * @return the generated command
     */
    public Command setAngleWithSpeed(double speed) {
        return this.run(() -> setAngleMotor(speed));
    }

    /**
     * create a command to set the angle motor's speed to the speed calculated by the pid controller
     * @return the generated command
     */
    public Command updateAngle() {
        return this.run(this::moveToTargetAngle);
    }

    /**
     * create a command to turn to a wanted angle
     * @param angle the desired angle
     * @return the generated command
     */
    public Command turnToAngle(double angle) {
        return this.
                runOnce(() -> setAngleSetpoint(angle)).
                andThen(run(this::moveToTargetAngle)).until(this::atSetpoint);
    }

    /**
     * create a command to aim at a wanted cube shelf
     * @param level the shelf level
     * @return the generated command
     */
    public Command turnToAngleVision(cuberConstants.angles level) {
        return this.
                runOnce(() -> setTargetAngleVision(level)).
                andThen(run(this::moveToTargetAngle)).until(this::atSetpoint);
    }

    /**
     * create a command to run the shooter with joysticks
     * @param speed the speed supplier
     * @return the generated command
     */
    public Command runWithJoysticks(DoubleSupplier speed) {
        return this.run(() -> this.setAngleMotor(speed.getAsDouble()));
    }
}

