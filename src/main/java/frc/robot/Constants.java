// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ShootingTables{
    public static final double[] dist = {0, 1.37, 1.7, 2.09, 2.32, 3.08, 3.81, 20};
    public static final double[] angle = {18.38, 18.38, 22.19, 25.76, 32.9, 37.67, 42.67, 42.67};
    public static final double[] velocity = {0,0,0,0,0,0,0,0};


  }

  public static class EFCConstants{
    public static final double intakeSpeed = 0.5;
    public static final double outtakeSpeed = -0.5;
    public static final double outtakeShooterVelocity = -1;

    public static final double ampShooterVelocity = 0;

    public static final double feedToShooterSpeed = 0.75;

  }

  public static class ArmConstants{
    public static final double duckingRad = Units.degreesToRadians(20.0);
    public static final double holdingRadSafe = Units.degreesToRadians(70.0);
    public static final double ampRad = Units.degreesToRadians(100);

    public static final double sourceRad = Units.degreesToRadians(95.0);
    public static final double intakeRad = Units.degreesToRadians(6);
  }



  public static class AutoConstants {
    public static final double translationkP = 2;
    public static final double rotationkP = 2;

    public static final double maxTranslation = 6.5; //no longer used
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

    public static final double alignmentkP = 1.5;

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

    public static final double turningMotorkP = 0.1;
    public static final double turningMotorkI = 0.0;
    public static final double turningMotorkD = 0.01;
    public static final double turningMotorkF = 0.0;


  }






}
