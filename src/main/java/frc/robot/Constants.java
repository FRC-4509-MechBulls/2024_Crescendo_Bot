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

    public static final double[] dist = {
            1.3,
            2,
            2.25,
            2.6,
            3,
            3.52,
            4.2
    };
    public static final double[] velocity = {
            50,
            60,
            60,
            60,
            70,
            70,
            70

    };
    public static final double[] angle = {
            25,
            38,
            38,
            43,
            44.5,
            45,
            49


    };


    /** //old head values


    public static final double[] dist = {
            1.31,
            2.33,
            2.56,
            2.77,
            3.16,
            3.51
    };
    public static final double[] velocity = {
            53,
            73,
            73,
            74,
            74,
            75
    };
    public static final double[] angle = {
            25,
            42.6,
            41,
            42.5,
            44.5,
            44

    };

     */



  }

  public static class EFCConstants{

    public static final double shooterkP = 0.00014617;
    public static final double shooterkI = 0;
    public static final double shooterkD = 0;
    public static final double shooterkV = 0.13621;
    public static final double shooterkA = 0.055923;


    public static final int intakeMasterID = 11;
    public static final int intakeFollowerID = 12;

    public static final int shooterMasterID = 23;
    public static final int shooterFollowerID = 22;

    public static final double intakeSpeed = 0.7;
    public static final double outtakeSpeed = -0.4;
    public static final double outtakeShooterVelocity = 0;

    public static final double ampShooterVelocity = 10;

    public static final double trapShooterVelocity = 80;

    public static final double feedToShooterSpeed = 1;

  }

  public static class ArmConstants{

    public static final double armMaxPower = 1; //0.6

    public static final double armkP = 2;
    public static final double armkI = 0.4;
    public static final double armkD = 0.0;


    public static final double armkPBrakeless = 0.3;
    public static final double armkIBrakeless = 0.1;
    public static final double armkDBrakeless = 0.0;

    public static final double brakeMinAngle = Units.degreesToRadians(-5);
    public static final double brakeMaxAngle = Units.degreesToRadians(130);


    public static final double armIZone = Units.degreesToRadians(10);

    public static final double brakeEngageError = Units.degreesToRadians(1);
    public static final double brakeDisengageError = Units.degreesToRadians(4);


    public static final double armGearRatio = (40.0/14) * 80.0;

    public static final int armMasterID = 50;
    public static final int armFollowerID = 31;
    public static final double duckingRad = Units.degreesToRadians(20.0);
    public static final double holdingRadSafe = Units.degreesToRadians(50.0);
    public static final double ampRad = Units.degreesToRadians(107.5);

    public static final double sourceRad = Units.degreesToRadians(95.0);
    public static final double intakeRad = Units.degreesToRadians(2);
  }

  public static class ClimbConstants{

    public static final double climbMaxPower = 1;
    public static final int climbMasterID = 50;
    public static final int climbFollowerID = 52;

    public static final double rotationsInTheClimbRange = 50; //73.2
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

    public static final double radFeedClamp = 1; //max heading adjustment speed
  }

  public static final class DriveConstants{

    public static final double alignmentkP = 12;

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
    public static final int frontLeftDriveID = 8;
    public static final int frontLeftTurningID = 2;
    public static final double frontLeftOffsetRad = 2.275 + Units.degreesToRadians(12);

    public static final int frontRightDriveID = 3;
    public static final int frontRightTurningID = 7;
    public static final double frontRightOffsetRad =  0.754;

    public static final int rearRightDriveID = 1;
    public static final int rearRightTurningID = 4;
    public static final double rearRightOffsetRad = 0.653 + + Units.degreesToRadians(16);

    public static final int rearLeftDriveID = 6;
    public static final int rearLeftTurningID = 5;
    public static final double rearLeftOffsetRad = 0.606 + Units.degreesToRadians(18);





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
