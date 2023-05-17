package frc.robot.subsystems;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.cuberConstants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;


    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;

    private final BangBangController shooterController;

    public Shooter() {
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
}

