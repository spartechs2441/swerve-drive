package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends Command {

    private final DriveSubsystem driveSub;
    private final XboxController joystick;
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
    public DriveCmd(DriveSubsystem driveSub, XboxController joystick, boolean fieldRelative, boolean rateLimit) {
        this.driveSub = driveSub;
        this.joystick = joystick;
        this.fieldRelative = fieldRelative;
        this.rateLimit = rateLimit;
        addRequirements(driveSub);
    }



    // Joystick Driving
    @Override
    public void execute() {
        boolean slow = this.joystick.getRawButton(Constants.Controls.slowDown);
        double xSpeed = -MathUtil.applyDeadband(this.joystick.getRawAxis(Constants.Controls.xMovement), Constants.OIConstants.kDriveDeadband);
        double ySpeed = -MathUtil.applyDeadband(this.joystick.getRawAxis(Constants.Controls.yMovement), Constants.OIConstants.kDriveDeadband);
        if (slow) {
            xSpeed /= 2;
            ySpeed /= 2;
        }
        double rotation = -MathUtil.applyDeadband(this.joystick.getRawAxis(Constants.Controls.rotation), Constants.OIConstants.kRotateDeadband);
        if (rotation > 1 || rotation < -1) {
            Robot.errorAssert("rotation cannot be > 1, < -1, actual value: " + rotation);
        }
        SmartDashboard.putNumber("Rotation", rotation * 360);

        this.driveSub.drive(xSpeed, ySpeed, rotation, this.fieldRelative, this.rateLimit);
    }
}

