package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TareCmd extends Command {

    private final DriveSubsystem driveSub;

    public TareCmd(DriveSubsystem driveSub) {
        this.driveSub = driveSub;
        addRequirements(driveSub);
    }
    @Override
    public void execute() {
        this.driveSub.tare();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
