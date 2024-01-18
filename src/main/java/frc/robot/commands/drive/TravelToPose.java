package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MBUtils;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class TravelToPose extends Command {
    private final SwerveSubsystem swerveSubsystem;

    Pose2d initialPose;
    Pose2d desiredPose;

    double initTime;
    double secondsToTake;
    double overtime;

    public TravelToPose(SwerveSubsystem swerveSubsystem, Pose2d desiredPose, double secondsToTake, double overtime) {
        this.swerveSubsystem = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

        this.desiredPose = desiredPose;
        this.secondsToTake = secondsToTake;
        this.overtime = overtime;
    }

    public TravelToPose (SwerveSubsystem swerveSubsystem, Pose2d desiredPose, double secondsToTake) {
        this(swerveSubsystem,desiredPose,secondsToTake,0);
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
        double aggression = 2;

        double interpolatedX = MBUtils.easeInOut(initialPose.getX(),desiredPose.getX(),(Timer.getFPGATimestamp() - initTime) / secondsToTake, aggression);
        double interpolatedY = MBUtils.easeInOut(initialPose.getY(),desiredPose.getY(),(Timer.getFPGATimestamp() - initTime) / secondsToTake,aggression);
        double interpolatedAngle = MBUtils.easeInOutSlerp(initialPose.getRotation().getRadians(), desiredPose.getRotation().getRadians(), (Timer.getFPGATimestamp() - initTime) / secondsToTake,aggression);

        Pose2d pPose = new Pose2d(interpolatedX,interpolatedY, Rotation2d.fromRadians(interpolatedAngle));


        Translation2d transDiff = desiredPose.getTranslation().minus(initialPose.getTranslation());
     //   double transSpeed = Math.hypot(transDiff.getX(), transDiff.getY()) / secondsToTake;
      //  double rotVelocity = desiredPose.getRotation().minus(initialPose.getRotation()).getRadians() / secondsToTake;

        /** new */
        double currentTime = (Timer.getFPGATimestamp() - initTime) / secondsToTake;
        double deltaT = 0.001; // small time increment
        double futureTime = currentTime + deltaT;

// Calculate current and future positions
        double currentX = MBUtils.easeInOut(initialPose.getX(), desiredPose.getX(), currentTime,aggression);
        double futureX = MBUtils.easeInOut(initialPose.getX(), desiredPose.getX(), futureTime,aggression);
        double currentY = MBUtils.easeInOut(initialPose.getY(), desiredPose.getY(), currentTime,aggression);
        double futureY = MBUtils.easeInOut(initialPose.getY(), desiredPose.getY(), futureTime,aggression);

// Calculate velocity
        double velocityX = (futureX - currentX) / (deltaT * secondsToTake);
        double velocityY = (futureY - currentY) / (deltaT * secondsToTake);

        double transSpeed = Math.hypot(velocityX,velocityY);

// Calculate current and future angles
        double currentAngle = MBUtils.easeInOutSlerp(initialPose.getRotation().getRadians(), desiredPose.getRotation().getRadians(), currentTime,aggression);
        double futureAngle = MBUtils.easeInOutSlerp(initialPose.getRotation().getRadians(), desiredPose.getRotation().getRadians(), futureTime,aggression);

// Calculate angular velocity
        double rotVelocity = (futureAngle - currentAngle) / (deltaT * secondsToTake);


/**End new*/

        double angleOfDiff = Math.atan2(transDiff.getY(),transDiff.getX()); //angle between two poses

        double angleOfTravel = angleOfDiff - interpolatedAngle;

        //SmartDashboard.putString()


        SmartDashboard.putNumber("transSpeed",transSpeed);

        //swerveSubsystem.drive(0,0,0);
        //swerveSubsystem.driveToPose(pPose);
        //swerveSubsystem.driveToPose(pPose,0,0,Math.cos(angleOfTravel)*transSpeed, Math.sin(angleOfTravel)*transSpeed, rotVelocity);

        swerveSubsystem.driveToPose(pPose,0.2,0.2,Math.cos(angleOfTravel)*transSpeed, Math.sin(angleOfTravel)*transSpeed, rotVelocity);



    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - initTime  > secondsToTake + overtime;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0,0,0);

    }
}
