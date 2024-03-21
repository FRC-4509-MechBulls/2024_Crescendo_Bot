package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.StateControllerSub;
import frc.robot.commands.state.ArmTrapPathInterpolation;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;

import java.util.ArrayList;
import java.util.List;

public class Alignments {


    // Load the path we want to pathfind to and follow
  //  static PathPlannerPath path = PathPlannerPath.fromPathFile("speaker-align");

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    static  PathConstraints constraints = new PathConstraints(4.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    static  PathConstraints slowConstraints = new PathConstraints(0.25, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands



    public static Command ampAlign = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("amp-align"),
            constraints,
            0
    );

    public static Command sourceAlign(){
        Pose2d pose = new Pose2d(14,1.1, Rotation2d.fromDegrees(180));
        pose = AllianceFlipUtil.apply(pose);
        return AutoBuilder.pathfindToPose(pose,constraints,0);

    }

    public static Command speakerAlign(){
        Pose2d pose = new Pose2d(1.75,FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(180));
        pose = AllianceFlipUtil.apply(pose);
        return AutoBuilder.pathfindToPose(pose,constraints,0);
    }

    public static Command trapTest(StateControllerSub stateControllerSub,Rotation2d angleToTravelIn){
        double distanceOutFar = 0.85;
        double distanceOutClose = 0.75;

        Pose2d startingPose = new Pose2d(FieldConstants.Stage.center.getX(),FieldConstants.Stage.center.getY(),Rotation2d.fromDegrees(180).plus(angleToTravelIn));

        Command retractEverything = new InstantCommand(()->stateControllerSub.setDuckMode(true))
                .andThen(new InstantCommand(()->stateControllerSub.setObjective(StateControllerSub.Objective.TRAP)))
                .andThen(new InstantCommand(()->stateControllerSub.setArmState(StateControllerSub.ArmState.TRAP)))
                .andThen(new InstantCommand(()->stateControllerSub.feedTrapArmAngle(Constants.ArmConstants.intakeRad)))
                .andThen(new InstantCommand(()->stateControllerSub.setClimbState(StateControllerSub.ClimbState.READY)));

        Command pathfindToInit = AutoBuilder.pathfindToPose(AllianceFlipUtil.apply(startingPose),constraints,0);




        Pose2d farthestPose = new Pose2d(startingPose.getX() + angleToTravelIn.getCos() * distanceOutFar, startingPose.getY() + angleToTravelIn.getSin() * distanceOutFar,angleToTravelIn.plus(Rotation2d.fromDegrees(180)));
        Pose2d closerPose = new Pose2d(startingPose.getX() + angleToTravelIn.getCos() * distanceOutClose, startingPose.getY() + angleToTravelIn.getSin() * distanceOutClose,angleToTravelIn.plus(Rotation2d.fromDegrees(180)));


        List<Translation2d> leaveBezier = PathPlannerPath.bezierFromPoses(
                new Pose2d(startingPose.getTranslation(),new Rotation2d()),
                new Pose2d(farthestPose.getTranslation(),new Rotation2d())
        );

        PathPlannerPath stageExitPath = new PathPlannerPath(leaveBezier,slowConstraints,new GoalEndState(0,farthestPose.getRotation()));

        Command followStageExit = AutoBuilder.followPath(stageExitPath);

        PathPlannerTrajectory stageExitTrajectory = stageExitPath.getTrajectory(new ChassisSpeeds(),startingPose.getRotation());
        Command armTrapPathInterpolation = new ArmTrapPathInterpolation(stateControllerSub,stageExitTrajectory,Units.degreesToRadians(20),Units.degreesToRadians(75));

        ParallelCommandGroup leaveStage = new ParallelCommandGroup(followStageExit,armTrapPathInterpolation);


        Command liftClimber = new InstantCommand(()->stateControllerSub.setClimbState(StateControllerSub.ClimbState.READY)).andThen(new InstantCommand(()->stateControllerSub.setDuckMode(false)));

        Command wait1 = new WaitCommand(0.75);

        List<Translation2d> backBezier = PathPlannerPath.bezierFromPoses(
                new Pose2d(farthestPose.getTranslation(),new Rotation2d()),
                new Pose2d(closerPose.getTranslation(),new Rotation2d())
        );
        PathPlannerPath againstChainPath = new PathPlannerPath(backBezier,slowConstraints,new GoalEndState(0,closerPose.getRotation()));

        Command driveAgainstChain = AutoBuilder.followPath(againstChainPath);

        Command doClimb = new InstantCommand(()->stateControllerSub.setClimbState(StateControllerSub.ClimbState.CLIMBED));

        Command climbUpSequence = retractEverything.andThen(pathfindToInit.andThen(leaveStage)).andThen(liftClimber).andThen(wait1).andThen(driveAgainstChain).andThen(doClimb);

        Command setToShootAngle = new InstantCommand(()->stateControllerSub.feedTrapArmAngle(Units.degreesToRadians(95)));

        Command readyToShoot = new InstantCommand(()->stateControllerSub.setEfState(StateControllerSub.EFState.READY));

        Command wait2 = new WaitCommand(1);

        Command shoot = new InstantCommand(()->stateControllerSub.setEfState(StateControllerSub.EFState.SHOOT));

        Command shootSequence = setToShootAngle.andThen(readyToShoot).andThen(wait2).andThen(shoot);







        return climbUpSequence.andThen(shootSequence);
    }



}
