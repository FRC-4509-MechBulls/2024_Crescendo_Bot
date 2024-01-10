// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FMSGetter;
import frc.robot.MBUtils;
import frc.robot.Robot;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.alignmentPoses;
import static frc.robot.Constants.FieldConstants.nodeYValues;
import static frc.robot.Constants.OperatorConstants.*;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  SwerveModule frontLeft = new SwerveModule(frontLeftDriveID,frontLeftTurningID,false,true,0,frontLeftOffsetRad);
  SwerveModule frontRight = new SwerveModule(frontRightDriveID,frontRightTurningID,false,true,1,frontRightOffsetRad);

  SwerveModule rearLeft = new SwerveModule(rearLeftDriveID,rearLeftTurningID,false,true,2,rearLeftOffsetRad);
  SwerveModule rearRight = new SwerveModule(rearRightDriveID,rearRightTurningID,false,true,3,rearRightOffsetRad);

  WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);
  AHRS navx = new AHRS(SPI.Port.kMXP);

  SwerveDrivePoseEstimator odometry;
VisionSubsystem visionSubsystem;

  Field2d field2d = new Field2d();

  Field2d visionField = new Field2d();

  boolean beFieldOriented = true;


  public SwerveSubsystem(VisionSubsystem visionSubsystem) {
    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    odometry = new SwerveDrivePoseEstimator(kinematics,pigeon.getRotation2d(),getPositions(),new Pose2d());
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(7, 7, Units.degreesToRadians(400)));
    this.visionSubsystem = visionSubsystem;

    SmartDashboard.putNumber("debugGoTo_x",0);
    SmartDashboard.putNumber("debugGoTo_y",0);
    SmartDashboard.putNumber("debugGoTo_deg",0);

  }

  public SwerveModulePosition[] getPositions(){
    return new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};
  }



  public void joystickDrive(double joystickX, double joystickY, double rad){

    if(Math.abs(rad)<controllerDeadband)
      rad = 0;


  rad*=1+controllerDeadband;
  if(rad>0)
    rad-=controllerDeadband;
  else if(rad<0)
    rad+=controllerDeadband;


  double hypot = Math.hypot(joystickX,joystickY);

  if(hypot<controllerDeadband)
    hypot = 0;
  hypot*=1+controllerDeadband;
  hypot-=controllerDeadband;
  if(hypot<0)
    hypot = 0;

  double dir = Math.atan2(joystickY,joystickX);

  //field oriented :p
    //oriented to 180 degrees
    double zeroHeading = 0;
    if(FMSGetter.isRedAlliance())
      zeroHeading = Math.PI; 

  if(beFieldOriented)
    dir+=odometry.getEstimatedPosition().getRotation().getRadians() + zeroHeading;

  hypot = Math.pow(hypot,driveExponent) * driveMaxSpeed;

  joystickX = hypot*Math.cos(dir);
  joystickY = hypot*Math.sin(dir);

  if(rad>0){
    rad = Math.pow(rad,turnExponent) * turnMaxSpeed;
  }else{
    rad = -Math.pow(-rad,turnExponent) * turnMaxSpeed;
  }


/* Angular adjustment stuff */
//  if(Math.abs(rad)>radPerSecondDeadband || lastStillHeading.getDegrees() == 0){
//    lastStillHeading = Rotation2d.fromDegrees(pigeon.getAngle());
//  }



  drive(-joystickY ,-joystickX ,-rad);
}




public void xConfig() {

  setStatesNoDeadband(new SwerveModuleState[]{
          new SwerveModuleState(0,Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0,Rotation2d.fromDegrees(45))

  });

}


Rotation2d lastStillHeading = new Rotation2d();
public void drive(double xMeters,double yMeters, double rad){
  if(Math.abs(rad)>radPerSecondDeadband || lastStillHeading.getDegrees() == 0){
    lastStillHeading = Rotation2d.fromDegrees(pigeon.getAngle());
  }

  double diffDeg = MBUtils.angleDiffDeg(pigeon.getAngle(),lastStillHeading.getDegrees());
  double radFeed = diffDeg * (1.0/25) ;  //this was responsible for the slap

  radFeed = MBUtils.clamp(radFeed,radFeedClamp);

  if(Math.abs(diffDeg)>25) radFeed = 0;
  if(!beFieldOriented) radFeed = 0;



  SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xMeters,yMeters,rad - radFeed));

  setStates(states);

  if(Robot.isSimulation()){
    if(lastSimDriveUpdateTime == 0)
      lastSimDriveUpdateTime = Timer.getFPGATimestamp();

    Rotation2d newAngle = Rotation2d.fromRadians(odometry.getEstimatedPosition().getRotation().getRadians() + rad * (Timer.getFPGATimestamp() - lastSimDriveUpdateTime));

    double hypot = Math.hypot(xMeters,yMeters);
    double angOfTranslation = Math.atan2(yMeters,xMeters);

    Pose2d newPose = new Pose2d(
            odometry.getEstimatedPosition().getX() + hypot*Math.cos(angOfTranslation+newAngle.getRadians()) *  (Timer.getFPGATimestamp() - lastSimDriveUpdateTime),
            odometry.getEstimatedPosition().getY() + hypot*Math.sin(angOfTranslation+newAngle.getRadians())  *(Timer.getFPGATimestamp() - lastSimDriveUpdateTime),
            newAngle
    );

  //  odometry.addVisionMeasurement(newPose, Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, Units.degreesToRadians(0))); //trust, bro

    resetOdometry(newPose);
    lastSimDriveUpdateTime = Timer.getFPGATimestamp();
  }

}
double lastSimDriveUpdateTime = 0;


  void setStates(SwerveModuleState[] states){
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);
  }

  void setStatesNoDeadband(SwerveModuleState[] states){
    frontLeft.setStateWithoutDeadband(states[0]);
    frontRight.setStateWithoutDeadband(states[1]);
    rearLeft.setStateWithoutDeadband(states[2]);
    rearRight.setStateWithoutDeadband(states[3]);
  }

  public void autoBalanceForward(){
    double roll = pigeon.getRoll(); //probably in degrees
    double speedForward = roll*(1.0/100);

    if(roll>15)
      speedForward = 0.5;
    if(roll<-15)
      speedForward = -0.5;


    speedForward = MBUtils.clamp(speedForward, 0.5);

    drive(-speedForward,0,0);
  }

void updatePoseFromVision(){
    Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(odometry.getEstimatedPosition());
    if(result.isPresent()){
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
      SmartDashboard.putNumber("lastVisionX",result.get().estimatedPose.getX());
      SmartDashboard.putNumber("lastVisionY",result.get().estimatedPose.getY());
      visionField.setRobotPose(result.get().estimatedPose.toPose2d());
      SmartDashboard.putData("visionField",visionField);


      SmartDashboard.putNumber("resultWasPresent",Timer.getFPGATimestamp());
    }
    //add vision measurement if present while passing in current reference pose
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("FLVel",frontLeft.getModuleVelocity());

odometry.updateWithTime(Timer.getFPGATimestamp(),pigeon.getRotation2d(),getPositions());

updatePoseFromVision();


  field2d.setRobotPose(odometry.getEstimatedPosition());
  //SmartDashboard.putData("field",field2d);

    SmartDashboard.putNumber("odom_x",odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odom_y",odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("odom_deg",odometry.getEstimatedPosition().getRotation().getDegrees());


    SmartDashboard.putNumberArray("odometry",new double[]{
            odometry.getEstimatedPosition().getX(),
            odometry.getEstimatedPosition().getY(),
            odometry.getEstimatedPosition().getRotation().getRadians()
    });




    SmartDashboard.putNumber("closestNodeY",getClosestNodeY());

  }

  public double getClosestNodeY(){
    double closestY = nodeYValues[0];
    double smallestDist = Math.abs(nodeYValues[0] - odometry.getEstimatedPosition().getY());

    for(int i = 0; i< nodeYValues.length; i++){
      double dist = Math.abs(nodeYValues[i] - odometry.getEstimatedPosition().getY());
      if(dist<smallestDist){
        smallestDist = dist;
        closestY = nodeYValues[i];
      }
    }

    return closestY;
  }

  public Pose2d getClosestNode(){
    double closestDist = Math.hypot(odometry.getEstimatedPosition().getX() - Constants.FieldConstants.alignmentPoses[0].getX(),odometry.getEstimatedPosition().getY() -  Constants.FieldConstants.alignmentPoses[0].getY());
    Pose2d output = Constants.FieldConstants.alignmentPoses[0];

    for(int i = 0; i<alignmentPoses.length; i++){
      double dist = Math.hypot(odometry.getEstimatedPosition().getX() - Constants.FieldConstants.alignmentPoses[i].getX(),odometry.getEstimatedPosition().getY() -  Constants.FieldConstants.alignmentPoses[i].getY());
      if(dist<closestDist){
        closestDist = dist;
        output = alignmentPoses[i];
      }
    }
    return output;
  }

  public void resetOdometry(){
    odometry.resetPosition(pigeon.getRotation2d(),getPositions(),new Pose2d());
  }


  public void resetOdometry(Pose2d newPose){
    odometry.resetPosition(pigeon.getRotation2d(),getPositions(),newPose);
  }


  public void toggleFieldOriented(){
    beFieldOriented = !beFieldOriented;
  }

  public void driveToPose(Pose2d desiredPose){
    driveToPose(desiredPose, maxTranslation,maxRotation,0,0,0);
  }

  public void driveToPose(Pose2d desiredPose, double maxSpeed, double maxRot,double xFF, double yFF, double radFF){
    Pose2d myPose = odometry.getEstimatedPosition();

    double xDiff = desiredPose.getX() - myPose.getX();
    double yDiff = desiredPose.getY() - myPose.getY();

//    double angDiff = desiredPose.getRotation().getRadians() - myPose.getRotation().getRadians(); //difference in angles

    double angDiff = -Units.degreesToRadians(MBUtils.angleDiffDeg(desiredPose.getRotation().getDegrees(), myPose.getRotation().getDegrees()));



    double hypot = Math.hypot(xDiff,yDiff);
    double angleOfDiff = Math.atan2(yDiff,xDiff); //angle between two poses

    double angleOfTravel = angleOfDiff - myPose.getRotation().getRadians();



    hypot*= translationkP;
    angDiff*=rotationkP;


    hypot = MBUtils.clamp(hypot,maxTranslation);
    hypot = MBUtils.clamp(hypot,maxSpeed);
    angDiff = MBUtils.clamp(angDiff,maxRotation);
    angDiff = MBUtils.clamp(angDiff,maxRot);




    drive( hypot*Math.cos(angleOfTravel) + xFF, hypot * Math.sin(angleOfTravel) + yFF, angDiff + radFF);

    SmartDashboard.putNumberArray("desiredPose",new double[]{desiredPose.getX(),desiredPose.getY(),desiredPose.getRotation().getRadians()});


  }



  public SwerveDrivePoseEstimator getOdometry(){
    return odometry;
  }
}
