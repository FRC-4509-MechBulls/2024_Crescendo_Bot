package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class WaitForArmError extends CommandBase {

    ArmSubsystem armSubsystem;
    double allowedError;
    double timeframe;
    double lastUnacceptableErrorTime = 0;

    public WaitForArmError(ArmSubsystem armSubsystem, double allowedError, double timeframe){
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.armSubsystem = armSubsystem;
        this.allowedError = allowedError;
        this.timeframe = timeframe;


        //addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        lastUnacceptableErrorTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Math.abs(armSubsystem.getArmError()) > allowedError)
            lastUnacceptableErrorTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - lastUnacceptableErrorTime > timeframe;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
