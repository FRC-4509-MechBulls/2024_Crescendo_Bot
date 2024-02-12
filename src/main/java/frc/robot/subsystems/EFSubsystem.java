package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

    CANSparkMax shooterMaster = new CANSparkMax(shooterMasterID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax shooterFollower = new CANSparkMax(shooterFollowerID, CANSparkMax.MotorType.kBrushless);


    public EFSubsystem(StateControllerSub stateControllerSub) {

        this.stateControllerSub = stateControllerSub;

        intakeMaster.configFactoryDefault(1000);
        intakeFollower.configFactoryDefault(1000);


        shooterMaster.restoreFactoryDefaults();
        shooterFollower.restoreFactoryDefaults();

        intakeFollower.follow(intakeMaster,FollowerType.PercentOutput);
        intakeFollower.setInverted(true);


        shooterFollower.follow(shooterMaster,true);

        shooterMaster.getPIDController().setP(0.00010225,0);
        shooterMaster.getPIDController().setI(0,0);
        shooterMaster.getPIDController().setD(0,0);

        shooterMaster.enableVoltageCompensation(12);



        shooterMaster.getEncoder().setVelocityConversionFactor(1.0/60);


        shooterMaster.burnFlash();

        SmartDashboard.putNumber("shooterVelocity",0);


    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("shooterVoltage",shooterMaster.getAppliedOutput() * shooterMaster.getVoltageCompensationNominalVoltage());



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
                if(stateControllerSub.getEFState() == StateControllerSub.EFState.SHOOT)
                    switch (stateControllerSub.getObjective()) {
                        case AMP -> flywheelVel = ampShooterVelocity;
                        case SPEAKER -> flywheelVel = stateControllerSub.getSpeakerFlywheelVel();
                        case TRAP -> {} //TODO: do something here?
                    }
                setMotors(feedToShooterSpeed,flywheelVel);
break;
        }

    }


    void setMotors(double intakePower, double shooterVelocity){ //positive is intake and shoot
        //TODO: implement this
    setShooterVelocity(SmartDashboard.getNumber("shooterVelocity",0));
    SmartDashboard.putNumber("shooterVelocityMeasured",shooterMaster.getEncoder().getVelocity());


    }


SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0,0.11746,0.037681);
    void setShooterVelocity(double velocity){
        double ff = feedforward.calculate(velocity);
        SmartDashboard.putNumber("ff",ff);
        shooterMaster.getPIDController().setReference(velocity, ControlType.kVelocity,0,ff);
    }



    void voltageDriveShooter(Measure<Voltage> voltage){
        shooterMaster.setVoltage(voltage.in(Volts));
    }

    void logShooterData(SysIdRoutineLog log){
        log.motor("shooter").voltage(Volts.of(shooterMaster.getAppliedOutput() * shooterMaster.getVoltageCompensationNominalVoltage())).angularVelocity(RotationsPerSecond.of(shooterMaster.getEncoder().getVelocity())).angularPosition(Rotations.of(shooterMaster.getEncoder().getPosition()));
    }

    SysIdRoutine shooterRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::voltageDriveShooter,this::logShooterData,this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return shooterRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return shooterRoutine.dynamic(direction);
    }


}

