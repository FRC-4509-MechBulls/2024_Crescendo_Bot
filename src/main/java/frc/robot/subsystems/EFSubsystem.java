package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.StateControllerSub;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EFCConstants.*;

public class EFSubsystem extends SubsystemBase {

    StateControllerSub stateControllerSub;

    TalonFX intakeMaster = new TalonFX(intakeMasterID);
    TalonFX intakeFollower = new TalonFX(intakeFollowerID);

    CANSparkMax upperShooter = new CANSparkMax(shooterMasterID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax lowerShooter = new CANSparkMax(shooterFollowerID, CANSparkMax.MotorType.kBrushless);


    public EFSubsystem(StateControllerSub stateControllerSub) {

        this.stateControllerSub = stateControllerSub;

        intakeMaster.configFactoryDefault(1000);
        intakeFollower.configFactoryDefault(1000);

        intakeMaster.configSupplyCurrentLimit(new com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration(true, 40, 45, 100));
        //TODO current limits?
        intakeFollower.follow(intakeMaster,FollowerType.PercentOutput);
        intakeFollower.setInverted(false);


        upperShooter.restoreFactoryDefaults();
        lowerShooter.restoreFactoryDefaults();

       upperShooter.setIdleMode(CANSparkBase.IdleMode.kBrake);
        lowerShooter.setIdleMode(CANSparkBase.IdleMode.kBrake);


      //  shooterFollower.follow(shooterMaster,true);


        upperShooter.getPIDController().setP(shooterkP,0);
        upperShooter.getPIDController().setI(shooterkI,0);
        upperShooter.getPIDController().setD(shooterkD,0);

        lowerShooter.getPIDController().setP(shooterkP,0);
        lowerShooter.getPIDController().setI(shooterkI,0);
        lowerShooter.getPIDController().setD(shooterkD,0);




        upperShooter.enableVoltageCompensation(12);
        lowerShooter.enableVoltageCompensation(12);

        upperShooter.setSmartCurrentLimit(40);
        lowerShooter.setSmartCurrentLimit(40);



        upperShooter.getEncoder().setVelocityConversionFactor(1.0/60);
        lowerShooter.getEncoder().setVelocityConversionFactor(1.0/60);

        upperShooter.setInverted(true);
        lowerShooter.setInverted(true);


        upperShooter.burnFlash();
        lowerShooter.burnFlash();

     //   SmartDashboard.putNumber("shooterVelocity",0);


    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("shooterVoltage", upperShooter.getAppliedOutput() * upperShooter.getVoltageCompensationNominalVoltage());
        SmartDashboard.putNumber("shooterVelocityMeasured", upperShooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("shooterVelocityMeasured_bottom", lowerShooter.getEncoder().getVelocity());



        // This method will be called once per scheduler run
        switch (stateControllerSub.getEFState()){
            case HOLD:
                setMotors(0,0);
                break;
            case INTAKE:
                setMotors(intakeSpeed,0);
                break;
            case EJECT:
                setMotors(outtakeSpeed, outtakeShooterVelocity);
                break;
            case READY:
            case SHOOT:
                double flywheelVel = 0;
                double intakeSpeed = 0;
                if(stateControllerSub.getEFState() == StateControllerSub.EFState.SHOOT)
                    intakeSpeed=feedToShooterSpeed;
                switch (stateControllerSub.getObjective()) {
                    case AMP -> flywheelVel = ampShooterVelocity;
                    case SPEAKER -> flywheelVel = stateControllerSub.getSpeakerFlywheelVel();
                    case TRAP -> flywheelVel = trapShooterVelocity;
                }
                setMotors(intakeSpeed,flywheelVel);
break;
        }

    }


    void setMotors(double intakePower, double shooterVelocity){ //positive is intake and shoot
        //TODO: implement this
        intakeMaster.set(ControlMode.PercentOutput,-intakePower);
        setShooterVelocity(shooterVelocity); //TODO re-enable shooter once it's not physically broken :)
        //   setShooterVelocity(SmartDashboard.getNumber("shooterVelocity",0));


    }


SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0,shooterkV,shooterkA);
    void setShooterVelocity(double velocity){
        double ff = feedforward.calculate(velocity);
        SmartDashboard.putNumber("ff",ff);
        upperShooter.getPIDController().setReference(velocity, ControlType.kVelocity,0,ff);
        lowerShooter.getPIDController().setReference(velocity, ControlType.kVelocity,0,ff);
    }



    void voltageDriveShooter(Measure<Voltage> voltage){
        upperShooter.setVoltage(voltage.in(Volts));
    }

    void logShooterData(SysIdRoutineLog log){
        log.motor("shooter").voltage(Volts.of(upperShooter.getAppliedOutput() * upperShooter.getVoltageCompensationNominalVoltage())).angularVelocity(RotationsPerSecond.of(upperShooter.getEncoder().getVelocity())).angularPosition(Rotations.of(upperShooter.getEncoder().getPosition()));
    }

    SysIdRoutine shooterRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::voltageDriveShooter,this::logShooterData,this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return shooterRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return shooterRoutine.dynamic(direction);
    }


}

