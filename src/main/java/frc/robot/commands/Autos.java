package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import static frc.robot.Constants.FieldConstants.*;

public class Autos {

    public static Command ballerAuto(SwerveSubsystem swerveSubsystem){
        Command resetPose = new InstantCommand(swerveSubsystem::resetOdometry);

        TravelToPose pose1 = new TravelToPose(swerveSubsystem, new Pose2d(0,2,Rotation2d.fromDegrees(90)),1.5,0);


        return resetPose.andThen(pose1);
       // return resetPose.andThen(pose1);
    }

}
