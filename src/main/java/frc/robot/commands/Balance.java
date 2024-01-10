package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MBUtils;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class Balance extends Command {
    private final SwerveSubsystem swerveSubsystem;


    double initTime;
    double secondsToTake;

    public Balance(SwerveSubsystem swerveSubsystem, double secondsToTake) {
        this.swerveSubsystem = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

        this.secondsToTake = secondsToTake;

    }



    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {



        swerveSubsystem.autoBalanceForward();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - initTime  > secondsToTake;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
