package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
StateControllerSub stateControllerSub;

    CANSparkMax climbPrimary = new CANSparkMax(climbMasterID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax climbSecondary = new CANSparkMax(climbFollowerID, CANSparkMax.MotorType.kBrushless);




PneumaticControlSub pneumaticControlSub;


    public ClimbSubsystem(StateControllerSub stateControllerSub,PneumaticControlSub pneumaticControlSub) {




//pcm.makeDoubleSolenoid(0,0).set();


        this.stateControllerSub = stateControllerSub;

        climbPrimary.restoreFactoryDefaults();
        climbSecondary.restoreFactoryDefaults();

        climbPrimary.setSmartCurrentLimit(40);
        climbSecondary.setSmartCurrentLimit(40);


        climbPrimary.getEncoder().setPosition(0);
        climbSecondary.getEncoder().setPosition(0);

        climbPrimary.getEncoder().setPositionConversionFactor(1.0/rotationsInTheClimbRange);
        climbSecondary.getEncoder().setPositionConversionFactor(1.0/rotationsInTheClimbRange);

        climbPrimary.getEncoder().setVelocityConversionFactor(1.0/rotationsInTheClimbRange/60);
        climbSecondary.getEncoder().setVelocityConversionFactor(1.0/rotationsInTheClimbRange/60);

        climbPrimary.getPIDController().setP(0.5);
        climbSecondary.getPIDController().setP(0.5);

        climbPrimary.getPIDController().setI(0.5);
        climbSecondary.getPIDController().setI(0.5);

        climbPrimary.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);
        climbSecondary.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);
//
        climbPrimary.getPIDController().setSmartMotionMaxVelocity(1,0);
        climbSecondary.getPIDController().setSmartMotionMaxVelocity(1,0);
//
        climbPrimary.getPIDController().setSmartMotionMaxAccel(0.5,0);
        climbSecondary.getPIDController().setSmartMotionMaxAccel(0.5,0);

        climbPrimary.getPIDController().setOutputRange(-climbMaxPower,climbMaxPower);
        climbSecondary.getPIDController().setOutputRange(-climbMaxPower,climbMaxPower);


      //  climbPrimary.getAlternateEncoder(AlternateEncoderType.kQuadrature,1);


      //  climbPrimary.getPIDController().setFeedbackDevice(climbPrimary.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,1));
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1,500);
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2,500);
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3,500);
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4,500);
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,500);
        climbPrimary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6,500);


        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1,500);
        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2,500);
        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3,500);
        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4,500);
        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,500);
        climbSecondary.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6,500);

        climbPrimary.enableVoltageCompensation(12);
        climbSecondary.enableVoltageCompensation(12);

        climbPrimary.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climbSecondary.setIdleMode(CANSparkBase.IdleMode.kBrake);

        climbPrimary.burnFlash();
        climbSecondary.burnFlash();

        this.pneumaticControlSub = pneumaticControlSub;
    }



    double setpointMeters = 0.0;//bottom
    double simMeters = 0.0;

    double climbAngle = 0;

    public double getClimbAngle(){
        return climbAngle;
    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simMeters += (setpointMeters - simMeters) * 0.1;

        switch (stateControllerSub.getClimbStateConsideringDuckMode()){
            case DOWN:
                extendPneumatic();
                 retractClaw();
                break;
            case READY:
                retractPneumatic();
                extendClaw();
                break;
            case CLIMBED:
                retractPneumatic();
                retractClaw();
                break;
        }

        NetworkTableInstance.getDefault().getTable("StateController").getEntry("clawPosition").setDouble(getClawPosition());
        NetworkTableInstance.getDefault().getTable("StateController").getEntry("climbStateConsideringDuckMode").setString(stateControllerSub.getClimbStateConsideringDuckMode().toString());


        if(stateControllerSub.getClimbStateConsideringDuckMode() == StateControllerSub.ClimbState.DOWN)
            climbAngle += (Units.degreesToRadians(-48) - climbAngle) * 0.1;
        else
            climbAngle += (Units.degreesToRadians(0) - climbAngle) * 0.1;

        stateControllerSub.feedClimbAngle(getClimbAngle());

    SmartDashboard.putNumber("climbMasterPosition", climbPrimary.getEncoder().getPosition());
    SmartDashboard.putNumber("climbFollowerPosition", climbSecondary.getEncoder().getPosition());

        SmartDashboard.putNumber("climbMasterVoltage",climbPrimary.getAppliedOutput()* climbPrimary.getVoltageCompensationNominalVoltage());
        SmartDashboard.putNumber("climbSecondaryVoltage",climbSecondary.getAppliedOutput()* climbSecondary.getVoltageCompensationNominalVoltage());

    }

    public double getClawPosition(){
        if(Robot.isSimulation())
            return simMeters;
        return climbPrimary.getEncoder().getPosition(); //TODO: actual claw encoder position
    }

    void extendPneumatic(){
        pneumaticControlSub.setClimbSolenoid(true);
    }
    void retractPneumatic(){
        pneumaticControlSub.setClimbSolenoid(false);
    }

    void extendClaw(){
        setpointMeters = 0;
        if(stateControllerSub.getClimbState() == StateControllerSub.ClimbState.DOWN)
            setpointMeters = 0.1;
        climbPrimary.getPIDController().setReference(-setpointMeters, ControlType.kPosition);
        climbSecondary.getPIDController().setReference(setpointMeters,ControlType.kPosition);
        climbPrimary.getPIDController().setIAccum(0);
        climbSecondary.getPIDController().setIAccum(0);
        SmartDashboard.putBoolean("clawExtended",true);
        //TODO: make it actually happen
    }

    void retractClaw(){
        setpointMeters = 1.0;

        climbPrimary.getPIDController().setReference(-setpointMeters, ControlType.kPosition);
        climbSecondary.getPIDController().setReference(setpointMeters,ControlType.kPosition);
        climbPrimary.getPIDController().setIAccum(0);
        climbSecondary.getPIDController().setIAccum(0);
        SmartDashboard.putBoolean("clawExtended",false);

        //todo: admire the windows 11 octopus emoji (hes so cute)
    }

}
