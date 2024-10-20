package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LockRotation extends Command {

    public final DriveSubsystem driveSub;
    private final Rotation2d targetRot;

    public LockRotation(DriveSubsystem driveSub, Rotation2d targetRot) {
        this.driveSub = driveSub;
        this.targetRot = targetRot;
        addRequirements(driveSub);
    }

    /**
     * TODO: Make this a log curve to get rid of overturning
     */
    @Override
    public void execute() {
        double difference = targetRot.getDegrees() % 360
                - driveSub.getGyro().getDegrees() % 360;
        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
        difference = (difference / 180);
        final double MIN_SPEED = 0.02;
        if (difference > 0) {
            difference = Math.max(difference, MIN_SPEED);
        } else if (difference < 0) {
            difference = Math.min(difference, -MIN_SPEED);
        }
        driveSub.drive(0, 0, difference, true, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
