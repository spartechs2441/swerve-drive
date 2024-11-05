package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends Command {

    private final DriveSubsystem driveSub;
    private final Joystick joystick;
    private final boolean fieldRelative;
    private final boolean rateLimit;

    /**
     * Method to drive the robot using joystick info.
     *
     * @param joystick The controller that would be effected by the
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public DriveCmd(DriveSubsystem driveSub, Joystick joystick, boolean fieldRelative, boolean rateLimit) {
        this.driveSub = driveSub;
        this.joystick = joystick;
        this.fieldRelative = fieldRelative;
        this.rateLimit = rateLimit;
        addRequirements(driveSub);
    }



    // Joystick Driving
    @Override
    public void execute() {
        double xSpeed = -MathUtil.applyDeadband(this.joystick.getRawAxis(1), Constants.OIConstants.kDriveDeadband);
        double ySpeed = -MathUtil.applyDeadband(this.joystick.getRawAxis(0), Constants.OIConstants.kDriveDeadband);
        double rotation = -MathUtil.applyDeadband(this.joystick.getRawAxis(2), Constants.OIConstants.kRotateDeadband);
        if (!this.joystick.getRawButton(1)) {
            rotation = 0;
        }
        if (rotation > 1 || rotation < -1) {
            Robot.errorAssert("rotation cannot be > 1, < -1, actual value: " + rotation);
        }
        SmartDashboard.putNumber("Rotation", rotation * 360);

        this.driveSub.drive(xSpeed, ySpeed, rotation, this.fieldRelative, this.rateLimit);
    }
}

