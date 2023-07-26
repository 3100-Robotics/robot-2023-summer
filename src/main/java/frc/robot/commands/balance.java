package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


public class balance extends CommandBase {
    private final Drive drive;

    double gyroReading, speed;
    boolean useRoll = false;

    /**
     * this command makes the robot balance in either rotation (with the front/back
     * on the charge station or with the left/right on the charge station)
     * @param swerveSubsystem the swerve drive to use
     */
    public balance(Drive swerveSubsystem) {
        this.drive = swerveSubsystem;

        // this command requires the drive subsystem
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        // determine if it is using roll or pitch
        if (drive.getRoll().getDegrees() > drive.getPitch().getDegrees()) {
            useRoll = true;
        }
    }

    @Override
    public void execute() {

        // take the input
        if (useRoll) {
            gyroReading = drive.getRoll().getDegrees();
        }
        else {
            gyroReading = drive.getPitch().getDegrees();
        }

        // calculate the speed with the pid controller
        speed = drive.calculate(gyroReading);

        // limit the speed
        if (Math.abs(speed) > 0.6) {
            speed = Math.copySign(0.6, speed);
        }

        // run the drivetrain
        if (useRoll) {
            drive.drive(new Translation2d(speed, 0), 0, false, false);
        }
        else {
            drive.drive(new Translation2d(0, speed), 0, false, false);
        }
    }

    @Override
    public boolean isFinished() {
        // the command is never done
        return false;
    }
}
