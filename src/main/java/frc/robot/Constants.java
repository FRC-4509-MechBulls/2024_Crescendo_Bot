// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class AutoConstants {
    public static final double translationkP = 4;
    public static final double rotationkP = 4;

    public static final double maxTranslation = 6.5;
    public static final double maxRotation = 2;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double driveExponent = 1.8;
    public static final double driveMaxSpeed = 5; //5
    public static final double turnExponent = 1.8;
    public static final double turnMaxSpeed = 11; //11


    public static final double maxDrivePower = 1;

    public static final double controllerDeadband = 0.06;

    public static final double radFeedClamp = 0.5; //max heading adjustment speed
  }

  public static final class DriveConstants{

    /*Physical Characteristics*/
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.625); //need to find
    // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(23.625); //need to find


    // Distance between front and back wheels
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );


    public static final double wheelDiameterMeters = Units.inchesToMeters(3.8);
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double driveMotorGearRatio = 6.75;
    public static final double turningGearRatio = 21.4286;
    public static final double falconTicks = 2048;

    public static final double radToFalcon = falconTicks / (2*Math.PI);

    public static final double radPerSecondDeadband = 0.01;




    /*Motor IDs and offsets */
    public static final int frontLeftDriveID = 1;
    public static final int frontLeftTurningID = 4;
    public static final double frontLeftOffsetRad = 0.945;

    public static final int frontRightDriveID = 6;
    public static final int frontRightTurningID = 5;
    public static final double frontRightOffsetRad = -5.1929;

    public static final int rearRightDriveID = 3;
    public static final int rearRightTurningID = 7;
    public static final double rearRightOffsetRad = 2.361;

    public static final int rearLeftDriveID = 8;
    public static final int rearLeftTurningID = 2;
    public static final double rearLeftOffsetRad = -2.296;





    /*Drive Motor Constants */
    public static final double driveMotorkP = 0.1;
    public static final double driveMotorkI = 0.0;
    public static final double driveMotorkD = 0.0;
    public static final double driveMotorkF = 0.045;

    public static final double driveNeutralDeadband = 0.01;



    /*Turning Motor Constants */

    public static final double turningMotorkP = 0.2;
    public static final double turningMotorkI = 0.0;
    public static final double turningMotorkD = 0.01;
    public static final double turningMotorkF = 0.0;


  }


  public static class ArmConstants {

    public static final double stageOnekP = 6;
    public static final double stageOnekI = 0.0;
    public static final double stageOnekD = 0.0;
    public static final double stageOnekF = 0.0;

    public static final double stageTwokP = 0.6; //0.8
    public static final double stageTwokI = 0.001; //0.001
    public static final double stageTwokD = 3; //2
    public static final double stageTwokF = 0.0;



    public static final int stageOneLeftId =  11;
    public static final int stageOneRightId = 13;

    public static final double continuousCurrentLimit = 20;
    public static final double peakCurrentLimit = 40;
    public static final double peakCurrentTime = 250;

    public static final int stageTwoPrimaryId = 49;
    public static final int stageTwoSecondaryId = 48;

    public static final int stageTwoSmartCurrentLimit = 40;
    public static final double stageTwoSecondaryCurrentLimit = 60;

    public static final double magEncoderCountsPerRotation = 4096;//4096
    public static final double radiansPerRotation = 2 * Math.PI;

    public static final double stageOneEncoderTicksToRadians =  (radiansPerRotation/magEncoderCountsPerRotation);

    public static final double stageOneEncoderOffset = Units.degreesToRadians(291.9 + 90 - .145);

    public static final double stageOneLength = Units.inchesToMeters(28.75);
    public static final double[] stageOnePivotCoordinate = {-4.864, 18.66};

    public static final double stageTwoLength = Units.inchesToMeters(28.75);
    public static final double stageTwoEncoderOffset = Units.degreesToRadians(43.6);//180 - 43.6 //43.6 + 180
    public static final double stageTwoEncoderRatio = 1;//32.0/22


   // public static final double[] affAnglesDegreesX = {-90,-73,-62,-54,-39,-30,-24,-17,-13,-11,-7,0,
   //         7,11,13,17,24,30,39,54,62,73,90};
  //  public static final double[] affPercentOutsY = {0.01,0.02,0.025,0.03,0.035,0.0375,0.04,0.042,0.043,0.046,0.047,
  //          0.047,0.046,0.043,0.042,0.04,0.0375,0.035,0.03,0.025,0.02,0.01,0};
  }

  public static class EfConstants { //end effector motor ids
    public static int EF_UPPER_PORT = 12;
    public static int EF_LOWER_PORT = 14;
  }


  public static final class FieldConstants{

    public static Pose2d[] alignmentPoses = new Pose2d[18];

    public static final double[] nodeYValues = new double[] {
            Units.inchesToMeters(20.19 + 22.0 * 0),
            Units.inchesToMeters(20.19 + 22.0 * 1),
            Units.inchesToMeters(20.19 + 22.0 * 2),
            Units.inchesToMeters(20.19 + 22.0 * 3),
            Units.inchesToMeters(20.19 + 22.0 * 4),
            Units.inchesToMeters(20.19 + 22.0 * 5),
            Units.inchesToMeters(20.19 + 22.0 * 6),
            Units.inchesToMeters(20.19 + 22.0 * 7),
            Units.inchesToMeters(20.19 + 22.0 * 8)
  };



    public static final double blueAlignmentX = Units.inchesToMeters(69.0625);
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double redAlignmentX = fieldLength - blueAlignmentX;

    static{
      for(int i = 0; i<9; i++){
        alignmentPoses[i] = new Pose2d(blueAlignmentX,nodeYValues[i], Rotation2d.fromDegrees(180));
        alignmentPoses[i+9] = new Pose2d(redAlignmentX,nodeYValues[i], Rotation2d.fromDegrees(0));
      }
    }

  }
}
