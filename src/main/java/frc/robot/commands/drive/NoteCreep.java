package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class NoteCreep extends CommandBase {

    SwerveSubsystem swerveSubsystem;
    double vel;
    double timeout;
    double startTime;
    public NoteCreep(SwerveSubsystem swerveSubsystem, double vel, double timeout) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerveSubsystem = swerveSubsystem;
        this.vel = vel;
        this.timeout = timeout;
        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize() {
this.startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {
        swerveSubsystem.noteAssistCreep(vel);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (Timer.getFPGATimestamp() - startTime) > timeout;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
