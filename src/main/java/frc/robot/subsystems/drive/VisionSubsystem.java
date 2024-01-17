package frc.robot.subsystems.drive;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

   // SendableChooser<PhotonPoseEstimator.PoseStrategy> selectedPoseStrategy = new SendableChooser<>();


    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera cam;
    PhotonPoseEstimator photonPoseEstimator;
    public VisionSubsystem() {

    //    SmartDashboard.putData("poseStrategy",selectedPoseStrategy);

        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }catch (Exception e){
            e.printStackTrace();
        }

        cam = new PhotonCamera("Arducam_OV2311_USB_Camera");
        Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(7), Units.inchesToMeters(0), Units.inchesToMeters(26)), new Rotation3d(0,Units.degreesToRadians(26),0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

        //1st + is cam forward
        //2nd + is cam to left
        //3rd + is cam up


    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
        
    }

    public void periodic(){
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);

    }


}

