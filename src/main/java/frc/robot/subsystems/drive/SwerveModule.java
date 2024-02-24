package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.maxDrivePower;

public class SwerveModule extends SubsystemBase {

    private WPI_TalonFX driveMotor;
    private WPI_TalonFX turningMotor;
    private DutyCycleEncoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    double absoluteEncoderOffset;


    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset){

        /*Motor Creation */
        driveMotor = new WPI_TalonFX(driveMotorID);
        turningMotor = new WPI_TalonFX(turningMotorID);
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID);

        driveMotor.configFactoryDefault();
        turningMotor.configFactoryDefault();

        /*Encoder Configs */
        absoluteEncoder.setDistancePerRotation(1.0);
        this.absoluteEncoderOffset = absoluteEncoderOffset;



        /*Drive Motor Configs */
        driveMotor.config_kP(0,driveMotorkP);
        driveMotor.config_kI(0,driveMotorkI);
        driveMotor.config_kD(0,driveMotorkD);
        driveMotor.config_kF(0,driveMotorkF);

        driveMotor.configVoltageCompSaturation(12.0);
        driveMotor.enableVoltageCompensation(true);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        StatorCurrentLimitConfiguration driveStatorConfig = new StatorCurrentLimitConfiguration(false,40,40,0);
        SupplyCurrentLimitConfiguration driveSupplyConfig = new SupplyCurrentLimitConfiguration(true,45,55,50);

        driveMotor.configStatorCurrentLimit(driveStatorConfig);
        driveMotor.configSupplyCurrentLimit(driveSupplyConfig);

        driveMotor.configNeutralDeadband(driveNeutralDeadband);

        driveMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setInverted(driveMotorReversed);

        driveMotor.configPeakOutputForward(maxDrivePower,1000);
        driveMotor.configPeakOutputReverse(-maxDrivePower,1000);

        /*Turning Motor Configs */
        turningMotor.config_kP(0,turningMotorkP);
        turningMotor.config_kI(0,turningMotorkI);
        turningMotor.config_kD(0,turningMotorkD);
        turningMotor.config_kF(0,turningMotorkF);

        turningMotor.configVoltageCompSaturation(12.0);
        turningMotor.enableVoltageCompensation(true);

        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        turningMotor.setInverted(true);



        StatorCurrentLimitConfiguration turningStatorConfig = new StatorCurrentLimitConfiguration(false,40,40,0);
        SupplyCurrentLimitConfiguration turningSupplyConfig = new SupplyCurrentLimitConfiguration(true,20,30,50);

        turningMotor.configStatorCurrentLimit(turningStatorConfig);
        turningMotor.configSupplyCurrentLimit(turningSupplyConfig);

        turningMotor.setNeutralMode(NeutralMode.Coast);
        turningMotor.setInverted(turningMotorReversed);

        new Thread(()->{
            try{
              //  for(int i = 0; i<10; i++){
                    Thread.sleep(1000);
                    driveMotor.setSelectedSensorPosition(0);
                    turningMotor.setSelectedSensorPosition(radToTurning(getAbsoluteEncoderRad()));
             //z   }
            } catch (InterruptedException e) {}
        }).start();
    }
    double getAbsoluteEncoderRad(){
        return absoluteEncoder.getAbsolutePosition()*2*Math.PI + absoluteEncoderOffset;
    }

    static double turningToRad(double turning){
        turning/=falconTicks;
        turning/=turningGearRatio;
        turning*= 2*Math.PI;
        return turning;
    }

    static double radToTurning(double rad){
        rad *= falconTicks;
        rad *= turningGearRatio;
        rad /= 2*Math.PI;
        return rad;
    }

    public double getAngle(){
        return turningToRad(turningMotor.getSelectedSensorPosition());
    }

    public void setState(SwerveModuleState state){

        if(state.speedMetersPerSecond<0.01){
            driveMotor.set(ControlMode.Velocity,0);
            return;
        }

        setStateWithoutDeadband(state);

    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getModuleVelocity(),Rotation2d.fromRadians(getAngle()));
    }

    public void setStateWithoutDeadband(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state,Rotation2d.fromRadians(getAngle())); //minimize change in heading

        double delta = state.angle.getRadians() - getAngle(); //error
        double deltaConverted = delta % Math.PI; //error converted to representative of the actual gap; error > pi indicates we aren't taking the shortest route to setpoint, but rather doing one or more 180* rotations.this is caused by the discontinuity of numbers(pi is the same location as -pi, yet -pi is less than pi)
        double setAngle = Math.abs(deltaConverted) < (Math.PI / 2) ? getAngle() + deltaConverted : getAngle() - ((deltaConverted/Math.abs(deltaConverted)) * (Math.PI-Math.abs(deltaConverted))); //makes set angle +/- 1/2pi of our current position(capable of pointing all directions)
    

        turningMotor.set(ControlMode.Position,radToTurning(setAngle));
        driveMotor.set(ControlMode.Velocity, driveMetersPerSecondToFalcon(state.speedMetersPerSecond));


    }

    public double driveMetersPerSecondToFalcon(double metersPerSecond){ //untested
        metersPerSecond /= 10; //convert from 1000ms to 100ms
        metersPerSecond = driveMetersToFalcon(metersPerSecond);
        return metersPerSecond;
    }

    public double driveMetersToFalcon(double meters){
        meters /= wheelCircumferenceMeters; //meters to rotations
        meters *= Constants.DriveConstants.falconTicks; //rotations to sensor units
        meters *= driveMotorGearRatio; //wheel rotations to motor rotations

        return meters;
    }

    public double falconToDriveMeters(double falconTicksIn){
        falconTicksIn *= wheelCircumferenceMeters;
        falconTicksIn /= Constants.DriveConstants.falconTicks;
        falconTicksIn /= driveMotorGearRatio;
        return falconTicksIn;
    }


    public SwerveModulePosition getPosition(){
      //  SmartDashboard.putNumber("sensorPosDrive",falconToDriveMeters(driveMotor.getSelectedSensorPosition()));
        return new SwerveModulePosition(falconToDriveMeters(driveMotor.getSelectedSensorPosition()),new Rotation2d(getAngle()));
    }

    public double getModuleVelocity(){
      //  return driveMotor.getSelectedSensorVelocity();
        return falconToDriveMeters(driveMotor.getSelectedSensorVelocity()) * 10;
    }



    public void periodic(){


    }

}