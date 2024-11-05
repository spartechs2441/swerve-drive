// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.command.DriveCmd;
import frc.robot.command.LockRotation;
import frc.robot.command.LimeLightCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem driveSub = new DriveSubsystem();
    public NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    // The driver's controller
    Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        // Configure default commands
        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveCmd(driveSub,
                        driverController,
                        true, true
                )
        );
    }

    public Rotation2d getGyroDirection() {
        return driveSub.getGyro();
    }

    public void printDangEncoders() {
        driveSub.printEncoders();
    }
    // if u alter this comment in any way u r gay

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Cease robot movement
//        new JoystickButton(driverController, 1).whileTrue(new RunCommand(robotDrive::forward, robotDrive));
        // new JoystickButton(driverController, 2).whileTrue(new RunCommand(robotDrive::resetEncoders, robotDrive));
        new JoystickButton(driverController, Constants.Controls.lockNorth).whileTrue(
                new LockRotation(driveSub, Rotation2d.fromDegrees(0))
        );
        new JoystickButton(driverController, Constants.Controls.lockSouth).whileTrue(
                new LockRotation(driveSub, Rotation2d.fromDegrees(180))
        );
        new JoystickButton(driverController, Constants.Controls.lockEast).whileTrue(
                new LockRotation(driveSub, Rotation2d.fromDegrees(90))
        );
        new JoystickButton(driverController, Constants.Controls.lockWest).whileTrue(
                new LockRotation(driveSub, Rotation2d.fromDegrees(270))
        );
        new JoystickButton(driverController, Constants.Controls.lightTrack).whileTrue(
                new LimeLightCmd(limeLight, driveSub, driverController)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
/*
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory, driveSub::getPose,
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, driveSub::setModuleStates, driveSub);

        // Reset odometry to the starting pose of the trajectory.
        driveSub.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> driveSub.drive(0, 0, 0, false, false));
*/
        return null;
    }
}
