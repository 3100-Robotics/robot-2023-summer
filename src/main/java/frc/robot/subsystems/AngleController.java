package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import frc.robot.Constants.visionConstants;
import frc.robot.Constants.cuberConstants;
import frc.robot.vision.visionWrapper;

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
    private final MechanismLigament2d claw = root.append(
            new MechanismLigament2d("claw", 9.8, 0));

    double setpoint = 0;

    /**
     * constructs a new angle controller and gives it two cameras.
     * @param frontCamera the camera in front
     * @param backCamera the camera in back
     */
    public AngleController(visionWrapper frontCamera, visionWrapper backCamera) {
        angleMotor = new CANSparkMax(cuberConstants.angleMotorPort, MotorType.kBrushless);

        // configure the motor
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(25);
        // to enable when I have the correct number of rotations
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)0.35);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        angleMotor.burnFlash();

        // set the encoder to be the connected absolute encoder
        angleEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        // set up the pid controller
        angleController = new PIDController(cuberConstants.angleP, cuberConstants.angleI, cuberConstants.angleD);
        angleController.setP(cuberConstants.angleP);
        angleController.setI(cuberConstants.angleI);
        angleController.setD(cuberConstants.angleD);

        // define the zero offset
        angleEncoder.setInverted(true);
        angleEncoder.setZeroOffset(0.9376072);


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
        SmartDashboard.putNumber("claw/angle setpoint", setpoint);
//        angleMotor.set(angleController.calculate(angleEncoder.getPosition()));
    }

    @Override
    public void simulationPeriodic() {
        // update the simulation and put the speed of the angle motor
        claw.setAngle(angleEncoder.getPosition()*360);
        REVPhysicsSim.getInstance().run();
        SmartDashboard.putNumber("angle speed", angleMotor.getEncoder().getVelocity());
    }

    // ACTIONS

    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    /**
     * set the setpoint angle
     * @param setpoint the wanted setpoint
     */
    public void setAngleSetpoint(double setpoint) {
        this.setpoint = setpoint;
        System.out.println("setting setpoint");
        angleController.setSetpoint(setpoint);
    }

    /**
     * set the angle of the shooter according to the wanted shelf to shoot too
     * @param level the shelf level
     */
    public void setTargetAngleVision(visionConstants.heights level) {
        // get the latest results
        PhotonPipelineResult frontResults = frontCamera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double distance = 0;

        // if there are results get the distance from them.
        // The front camera is prioritized
        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        // calculate the angle and set it as the setpoint
        setAngleSetpoint((Math.atan(
                (2/distance) *
                (level.getHeightDiff() + Constants.visionConstants.maxHeight +
                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                level.getHeightDiff() *
                Constants.visionConstants.maxHeight)))+(Math.PI/2))/2*Math.PI);
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

    public double getSetpoint() {
        return setpoint;
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
     * create a command to turn to a wanted angle
     * @param angle the desired angle
     * @return the generated command
     */
    public Command turnToAngle(double angle) {
        return this.
                runOnce(() -> setAngleSetpoint(angle));
    }

    /**
     * create a command to aim at a wanted cube shelf
     * @param level the shelf level
     * @return the generated command
     */
    public Command turnToAngleVision(visionConstants.heights level) {
        return this.
                runOnce(() -> setTargetAngleVision(level));
    }

    /**
     * create a command to run the shooter with joysticks
     * @param speed the speed supplier
     * @return the generated command
     */
    public Command runWithJoysticks(DoubleSupplier speed) {
        return this.run(() -> this.setAngleMotor(-0.4*speed.getAsDouble()));
    }
}

