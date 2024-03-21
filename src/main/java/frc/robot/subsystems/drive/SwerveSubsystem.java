// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;
import frc.robot.util.MBUtils;
import frc.robot.Robot;
import frc.robot.StateControllerSub;
import org.photonvision.EstimatedRobotPose;

import java.lang.reflect.Field;
import java.util.Optional;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import static frc.robot.Constants.OperatorConstants.*;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  SwerveModule frontLeft = new SwerveModule(frontLeftDriveID,frontLeftTurningID,false,true,3,frontLeftOffsetRad);
  SwerveModule frontRight = new SwerveModule(frontRightDriveID,frontRightTurningID,true,true,1,frontRightOffsetRad);

  SwerveModule rearLeft = new SwerveModule(rearLeftDriveID,rearLeftTurningID,true,true,0,rearLeftOffsetRad);
  SwerveModule rearRight = new SwerveModule(rearRightDriveID,rearRightTurningID,true,true,2,rearRightOffsetRad);

  WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);
  AHRS navx = new AHRS(SPI.Port.kMXP);

  SwerveDrivePoseEstimator odometry;
VisionSubsystem visionSubsystem;
StateControllerSub stateController;


  Field2d field2d = new Field2d();

  Field2d visionField = new Field2d();

  boolean beFieldOriented = true;


  public SwerveSubsystem(VisionSubsystem visionSubsystem, StateControllerSub stateController) {
    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    odometry = new SwerveDrivePoseEstimator(kinematics,pigeon.getRotation2d(),getPositions(),new Pose2d());
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, Units.degreesToRadians(400)));
    this.visionSubsystem = visionSubsystem;
    this.stateController = stateController;

  //  SmartDashboard.putNumber("debugGoTo_x",0);
  //  SmartDashboard.putNumber("debugGoTo_y",0);
  //  SmartDashboard.putNumber("debugGoTo_deg",0);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    PPHolonomicDriveController.setRotationTargetOverride(stateController::getRotationTargetOverride);
  //  PPLibTelemetry.
  }


  public void drive(ChassisSpeeds speeds){
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public Pose2d getPose(){
    return odometry.getEstimatedPosition();
  }

  ChassisSpeeds getRobotRelativeSpeeds(){
    if(Robot.isSimulation())
      return new ChassisSpeeds(simXMeters,simYMeters,simRad);
    return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
  }


  public SwerveModulePosition[] getPositions(){
    return new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};
  }



  public void joystickDrive(double joystickX, double joystickY, double rad){

    double rawJoyHypot = Math.hypot(joystickX,joystickY);

    if(Math.abs(rad)<controllerDeadband)
      rad = 0;


    SmartDashboard.putNumber("rad",rad);


    if(Math.abs(rad)<controllerDeadband)
      rad = 0;
 // rad*=1+controllerDeadband;
 // if(rad>0)
 //   rad-=controllerDeadband;
 // else if(rad<0)
 //   rad+=controllerDeadband;


  double hypot = Math.hypot(joystickX,joystickY);

  if(hypot<controllerDeadband)
    hypot = 0;
  hypot*=1+controllerDeadband;
  hypot-=controllerDeadband;
  if(hypot<0)
    hypot = 0;

  double dir = Math.atan2(joystickY,joystickX);

    if(rad>0){
      rad = Math.pow(rad,turnExponent) * turnMaxSpeed;
    }else{
      rad = -Math.pow(-rad,turnExponent) * turnMaxSpeed;
    }

    hypot = Math.pow(hypot,driveExponent) * driveMaxSpeed;

  //field oriented :p
    //oriented to 180 degrees
    double zeroHeading = 0;
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
      zeroHeading = Math.PI; 

  if(beFieldOriented)
    dir+=odometry.getEstimatedPosition().getRotation().getRadians() + zeroHeading;



  joystickX = hypot*Math.cos(dir);
  joystickY = hypot*Math.sin(dir);



    if(stateController.alignWhenClose() && stateController.getArmState() != StateControllerSub.ArmState.HOLD)
      rad+=MBUtils.clamp(stateController.alignWhenCloseAngDiff() * alignmentkP,1);

    double creep = 0;

    if(stateController.getArmState() == StateControllerSub.ArmState.INTAKE && rawJoyHypot<0.1)
     creep = -0.5 ; //-0.5

    SmartDashboard.putNumber("rawJoyHypot",rawJoyHypot);

    SmartDashboard.putNumber("alignWhenCloseAngDiff",stateController.alignWhenCloseAngDiff());



    /* Angular adjustment stuff */
//  if(Math.abs(rad)>radPerSecondDeadband || lastStillHeading.getDegrees() == 0){
//    lastStillHeading = Rotation2d.fromDegrees(pigeon.getAngle());
//  }



  drive(-joystickY + creep ,-joystickX ,-rad);
}

public void noteAssistCreep(double vel){
    double rad = MBUtils.clamp(stateController.alignWhenCloseAngDiff() * alignmentkP,1);
    drive(0,vel,-rad);
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

  if(Robot.isSimulation()){
    simXMeters = xMeters;
    simYMeters = yMeters;
    simRad = rad;

  }

/*
  if(Math.abs(rad)>radPerSecondDeadband || lastStillHeading.getDegrees() == 0){
    lastStillHeading = Rotation2d.fromDegrees(pigeon.getAngle());
  }

  double diffDeg = MBUtils.angleDiffDeg(pigeon.getAngle(),lastStillHeading.getDegrees());
  double radFeed = diffDeg * (1.0/25) ;  //this was responsible for the slap

  radFeed = MBUtils.clamp(radFeed,radFeedClamp);

  if(Math.abs(diffDeg)>25) radFeed = 0;
  if(!beFieldOriented) radFeed = 0;


  radFeed = 0;
 //TODO subtract radfeed if it should be enabled
 */
  SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xMeters,yMeters,rad ));

  SmartDashboard.putNumberArray("fieldsPassedIntoKinematics",new double[]{xMeters,yMeters,rad});
  setStates(states);



}
double lastSimDriveUpdateTime = 0;

double simXMeters,simYMeters,simRad;
void simDriveUpdate(){
  if(lastSimDriveUpdateTime == 0)
    lastSimDriveUpdateTime = Timer.getFPGATimestamp();

  Rotation2d newAngle = Rotation2d.fromRadians(odometry.getEstimatedPosition().getRotation().getRadians() + simRad * (Timer.getFPGATimestamp() - lastSimDriveUpdateTime));

  double hypot = Math.hypot(simXMeters,simYMeters);
  double angOfTranslation = Math.atan2(simYMeters,simXMeters);

  Pose2d newPose = new Pose2d(
          odometry.getEstimatedPosition().getX() + hypot*Math.cos(angOfTranslation+newAngle.getRadians()) *  (Timer.getFPGATimestamp() - lastSimDriveUpdateTime),
          odometry.getEstimatedPosition().getY() + hypot*Math.sin(angOfTranslation+newAngle.getRadians())  *(Timer.getFPGATimestamp() - lastSimDriveUpdateTime),
          newAngle
  );

  //  odometry.addVisionMeasurement(newPose, Timer.getFPGATimestamp(), VecBuilder.fill(0, 0, Units.degreesToRadians(0))); //trust, bro

  resetOdometry(newPose);
  lastSimDriveUpdateTime = Timer.getFPGATimestamp();
}

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





void updatePoseFromVision(){
    Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(odometry.getEstimatedPosition());
    if(result.isPresent()){
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    //  SmartDashboard.putNumber("lastVisionX",result.get().estimatedPose.getX());
    //  SmartDashboard.putNumber("lastVisionY",result.get().estimatedPose.getY());
      visionField.setRobotPose(result.get().estimatedPose.toPose2d());
    //  SmartDashboard.putData("visionField",visionField);


      Pose3d odometry3D = result.get().estimatedPose;
      SmartDashboard.putNumberArray("vision3D",new double[]{odometry3D.getX(),odometry3D.getY(),odometry3D.getZ(),odometry3D.getRotation().getQuaternion().getW(),odometry3D.getRotation().getQuaternion().getX(),odometry3D.getRotation().getQuaternion().getY(),odometry3D.getRotation().getQuaternion().getZ()});



    //  SmartDashboard.putNumber("resultWasPresent",Timer.getFPGATimestamp());
    }
    //add vision measurement if present while passing in current reference pose
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateController.feedRobotPose(odometry.getEstimatedPosition());

    //SmartDashboard.putNumber("frontLeftDriveVel",frontLeft.getState().speedMetersPerSecond);
    //SmartDashboard.putNumber("frontLeftDriveVel2",frontLeft.getModuleVelocity());

   // SmartDashboard.putNumber("frontLeftDrivePos",frontLeft.getPosition().distanceMeters);

    SmartDashboard.putNumberArray("modulePositions",new double[]{frontLeft.getPosition().distanceMeters,frontRight.getPosition().distanceMeters,rearLeft.getPosition().distanceMeters,rearRight.getPosition().distanceMeters});
    SmartDashboard.putNumberArray("moduleHeadings",new double[]{frontLeft.getAngle(),frontRight.getAngle(),rearLeft.getAngle(),rearRight.getAngle()});
    SmartDashboard.putNumberArray("moduleAbsoluteReadings",new double[]{frontLeft.getAbsoluteEncoderRad(),frontRight.getAbsoluteEncoderRad(),rearLeft.getAbsoluteEncoderRad(),rearRight.getAbsoluteEncoderRad()});
    SmartDashboard.putNumberArray("moduleAbsoluteReadingsWrappedToMatch",new double[]{frontLeft.getAbsoluteEncoderRadWrappedToMatch(),frontRight.getAbsoluteEncoderRadWrappedToMatch(),rearLeft.getAbsoluteEncoderRadWrappedToMatch(),rearRight.getAbsoluteEncoderRadWrappedToMatch()});

 //   SmartDashboard.putNumber("Mathdotrandom",Math.random());

    if(Robot.isSimulation())
      simDriveUpdate();

    if(Robot.isReal())
      odometry.updateWithTime(Timer.getFPGATimestamp(),pigeon.getRotation2d(),getPositions());

    updatePoseFromVision();

    SmartDashboard.putNumberArray("odometry",new double[]{
            odometry.getEstimatedPosition().getX(),
            odometry.getEstimatedPosition().getY(),
            odometry.getEstimatedPosition().getRotation().getRadians()
    });

  //  SmartDashboard.putNumber("fl_head",frontLeft.getAbsoluteEncoderRad());
  //  SmartDashboard.putNumber("fr_head",frontRight.getAbsoluteEncoderRad());
  //  SmartDashboard.putNumber("rl_head",rearLeft.getAbsoluteEncoderRad());
 //   SmartDashboard.putNumber("rr_head",rearRight.getAbsoluteEncoderRad());



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

 //   SmartDashboard.putNumberArray("desiredPose",new double[]{desiredPose.getX(),desiredPose.getY(),desiredPose.getRotation().getRadians()});


  }



  public SwerveDrivePoseEstimator getOdometry(){
    return odometry;
  }
}
