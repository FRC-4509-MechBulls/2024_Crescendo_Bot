package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.TravelToPose;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Autos {

    public static Command pathPlannerTest(SwerveSubsystem swerveSubsystem){
        var path = PathPlannerPath.fromPathFile("New New Path").getTrajectory(new ChassisSpeeds(0,0,0),Rotation2d.fromDegrees(0));

        Command resetPose = new InstantCommand(()->swerveSubsystem.resetOdometry(path.getInitialTargetHolonomicPose()));
        Command trajectoryCommand = new FollowTrajectory(swerveSubsystem, path);

        return resetPose.andThen(trajectoryCommand);

    }



    public static Command ballerAuto(SwerveSubsystem swerveSubsystem){
        Command resetPose = new InstantCommand(swerveSubsystem::resetOdometry);

        TravelToPose pose1 = new TravelToPose(swerveSubsystem, new Pose2d(0,2,Rotation2d.fromDegrees(90)),4,0);


        return resetPose.andThen(pose1);
       // return resetPose.andThen(pose1);
    }

}
