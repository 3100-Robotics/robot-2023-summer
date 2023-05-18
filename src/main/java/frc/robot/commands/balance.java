package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveSubsystem;


public class balance extends CommandBase {
    private final swerveSubsystem drive;

    double gyroReading, speed;
    boolean useRoll = false;

    public balance(swerveSubsystem swerveSubsystem) {
        this.drive = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        if (drive.getRoll().getDegrees() > drive.getPitch().getDegrees()) {
            useRoll = true;
        }
    }

    @Override
    public void execute() {
        if (useRoll) {
            gyroReading = drive.getRoll().getDegrees();
        }
        else {
            gyroReading = drive.getPitch().getDegrees();
        }
        speed = drive.calculate(gyroReading);
        if (Math.abs(speed) > 0.6) {
            speed = Math.copySign(0.6, speed);
        }
        if (useRoll) {
            drive.drive(new Translation2d(speed, 0), 0, false, false);
        }
        else {
            drive.drive(new Translation2d(0, speed), 0, false, false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
