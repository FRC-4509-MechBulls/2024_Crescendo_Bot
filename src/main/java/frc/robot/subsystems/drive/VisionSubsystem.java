package frc.robot.subsystems.drive;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateControllerSub;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

   // SendableChooser<PhotonPoseEstimator.PoseStrategy> selectedPoseStrategy = new SendableChooser<>();


    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera shooterSideArducam;

    PhotonCamera moduleArducam;

    PhotonCamera intakeAssistCamera = new PhotonCamera("intake_assist_awesome");
    PhotonPoseEstimator photonPoseEstimatorShooter;

    PhotonPoseEstimator photonPoseEstimatorModule;

    StateControllerSub stateControllerSub;
    public VisionSubsystem(StateControllerSub stateControllerSub) {

    //    SmartDashboard.putData("poseStrategy",selectedPoseStrategy);
        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }catch (Exception e){
            e.printStackTrace();
        }

        shooterSideArducam = new PhotonCamera("Arducam_OV2311_shooter");
        moduleArducam = new PhotonCamera("Arducam_OV2311_module2");
        Transform3d robotToShooterCam = new Transform3d(new Translation3d(0.357,0.076,0.224), new Rotation3d(0,Units.degreesToRadians(-30),0)); //z = 0.224
        Transform3d robotToModuleCam = new Transform3d(new Translation3d(-0.355,-0.335,0.152), new Rotation3d(0,Units.degreesToRadians(-30),Units.degreesToRadians(180)));

        //use max dist for lens?
        //x -> y
        //y -> -x
        //z -> -z


        photonPoseEstimatorShooter = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterSideArducam, robotToShooterCam);
        photonPoseEstimatorModule = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, moduleArducam, robotToModuleCam);
       // photonPoseEstimator.setPrimaryStrategy();
        photonPoseEstimatorShooter.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorModule.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);


        //1st + is cam forward
        //2nd + is cam to left
        //3rd + is cam up

        this.stateControllerSub = stateControllerSub;

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {

        Optional<EstimatedRobotPose> shooterEstimate = photonPoseEstimatorShooter.update();
        Optional<EstimatedRobotPose> moduleEstimate = photonPoseEstimatorModule.update();


        if(shooterEstimate.isPresent())
            return shooterEstimate;
        return moduleEstimate;
    }




    public void periodic(){

      List<PhotonTrackedTarget> targets = intakeAssistCamera.getLatestResult().getTargets();
      double assistAngle = 0;
      if(!targets.isEmpty())
          assistAngle = Units.degreesToRadians(targets.get(0).getYaw()) * 0.6;
      stateControllerSub.feedNoteAlignAngleDiff(assistAngle);

        SmartDashboard.putNumber("noteAssistAngle",assistAngle);


    }


}

