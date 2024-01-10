package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MBUtils;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class FollowTrajectory extends Command {
    private final SwerveSubsystem swerveSubsystem;

    Pose2d initialPose;
    PathPlannerTrajectory trajectory;

    double initTime;

    double secondsToTake;
    public FollowTrajectory(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory trajectory) {
        this.swerveSubsystem = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

        this.trajectory = trajectory;
        this.secondsToTake = trajectory.getTotalTimeSeconds();

    }


    @Override
    public void initialize() {
        initialPose = swerveSubsystem.getOdometry().getEstimatedPosition();
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - initTime > secondsToTake){
        //    swerveSubsystem.driveToPose(desiredPose);
            return;
        }
       // double aggression = 2;

        double currentTime = (Timer.getFPGATimestamp() - initTime);

        Pose2d sampledPoseNow = trajectory.sample(currentTime).getTargetHolonomicPose();
        double currentX = sampledPoseNow.getX();
        double currentY = sampledPoseNow.getY();
        double currentAngle = sampledPoseNow.getRotation().getRadians();

        /** new */

        double deltaT = 0.001; // small time increment
        double futureTime = currentTime + deltaT;

// Calculate current and future positions
        Pose2d sampledPoseFuture = trajectory.sample(futureTime).getTargetHolonomicPose();
        double futureX = sampledPoseFuture.getX();
        double futureY = sampledPoseFuture.getY();
        double futureAngle = sampledPoseFuture.getRotation().getRadians();

// Calculate velocity
        double velocityX = (futureX - currentX) / (deltaT );
        double velocityY = (futureY - currentY) / (deltaT );

        double transSpeed = Math.hypot(velocityX,velocityY);

// Calculate current and future angles

// Calculate angular velocity
        double rotVelocity = (futureAngle - currentAngle) / (deltaT);


/**End new*/

  //old      double angleOfDiff = Math.atan2(transDiff.getY(),transDiff.getX()); //angle between two poses
        double fieldRelativeAngleOfTravel = Math.atan2(futureY - currentY, futureX - currentX);

        double angleOfTravel = fieldRelativeAngleOfTravel - currentAngle;

        //SmartDashboard.putString()


        SmartDashboard.putNumber("transSpeed",transSpeed);

        //swerveSubsystem.drive(0,0,0);
        //swerveSubsystem.driveToPose(pPose);
        //swerveSubsystem.driveToPose(pPose,0,0,Math.cos(angleOfTravel)*transSpeed, Math.sin(angleOfTravel)*transSpeed, rotVelocity);

        swerveSubsystem.driveToPose(sampledPoseNow,0.2,0.2,Math.cos(angleOfTravel)*transSpeed, Math.sin(angleOfTravel)*transSpeed, rotVelocity);



    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - initTime  > secondsToTake;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0,0,0);

    }
}
