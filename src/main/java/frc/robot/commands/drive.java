package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import swervelib.SwerveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class drive extends CommandBase {

    private final Drive swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final boolean isOpenLoop;
    private final SwerveController controller;
    private final Timer timer = new Timer();
    private final boolean headingCorrection;
    private double angle = 0;
    private double lastTime = 0;

    /**
     * constructs a command to drive the robot in either robot-centric or field-centric mode
     * @param swerve the drive subsystem
     * @param vX the x velocity supplier
     * @param vY the y velocity supplier
     * @param omega the rotation supplier
     * @param driveMode whether it should drive in robot-centric or field-centric
     * @param isOpenLoop whether it should drive in open loop mode
     * @param headingCorrection whether it should correct the heading
     */
    public drive(Drive swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                 BooleanSupplier driveMode, boolean isOpenLoop, boolean headingCorrection) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerve.getSwerveController();
        this.headingCorrection = headingCorrection;
        if (headingCorrection) {
            timer.start();
        }

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (headingCorrection) {
            lastTime = timer.get();
        }
    }

    @Override
    public void execute() {
        // cube teh inputs for more controllability
        double xVelocity = Math.pow(vX.getAsDouble(), 3);
        double yVelocity = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);

        if (headingCorrection) {
            angle += (angVelocity * (timer.get() - lastTime)) * controller.config.maxAngularVelocity;

            ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVelocity, yVelocity, angle,
                    swerve.getHeading().getRadians());

            swerve.drive(
                    SwerveController.getTranslation2d(correctedChassisSpeeds),
                    correctedChassisSpeeds.omegaRadiansPerSecond,
                    driveMode.getAsBoolean(),
                    isOpenLoop);

            lastTime = timer.get();
        }
        else {
            swerve.drive(new Translation2d(
                            xVelocity * controller.config.maxSpeed,
                            yVelocity * controller.config.maxSpeed),
                    angVelocity * controller.config.maxAngularVelocity,
                    driveMode.getAsBoolean(), isOpenLoop);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
