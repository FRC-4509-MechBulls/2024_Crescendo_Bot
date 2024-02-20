package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
StateControllerSub stateControllerSub;

    CANSparkMax climbMaster = new CANSparkMax(climbMasterID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax climbFollower = new CANSparkMax(climbFollowerID, CANSparkMax.MotorType.kBrushless);



    public ClimbSubsystem(StateControllerSub stateControllerSub) {

        this.stateControllerSub = stateControllerSub;

        climbMaster.restoreFactoryDefaults();
        climbFollower.restoreFactoryDefaults();

        climbMaster.setSmartCurrentLimit(40);
        climbFollower.setSmartCurrentLimit(40);

        climbMaster.getEncoder().setPosition(0);
        climbFollower.getEncoder().setPosition(0);

        climbMaster.getEncoder().setPositionConversionFactor(1.0/rotationsInTheClimbRange);
        climbFollower.getEncoder().setPositionConversionFactor(1.0/rotationsInTheClimbRange);

        climbMaster.getEncoder().setVelocityConversionFactor(1.0/rotationsInTheClimbRange/60);
        climbFollower.getEncoder().setVelocityConversionFactor(1.0/rotationsInTheClimbRange/60);

        climbMaster.getPIDController().setP(0.2);
        climbFollower.getPIDController().setP(0.2);

        climbMaster.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);
        climbFollower.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);

        climbMaster.getPIDController().setSmartMotionMaxVelocity(0.5,0);
        climbMaster.getPIDController().setSmartMotionMaxVelocity(0.5,0);

        climbMaster.getPIDController().setSmartMotionMaxAccel(0.5,0);
        climbMaster.getPIDController().setSmartMotionMaxAccel(0.5,0);

        climbMaster.getPIDController().setOutputRange(-0.2,0.2);
        climbFollower.getPIDController().setOutputRange(-0.2,0.2);


        climbMaster.burnFlash();
        climbFollower.burnFlash();
    }

    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    static final double maxClawDistanceMeters = 0.50;//TODO: find this
    double setpointMeters = 0.0;//bottom
    double simMeters = 0.0;


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simMeters += (setpointMeters - simMeters) * 0.1;

        switch (stateControllerSub.getClimbState()){
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

    }

    public double getClawPosition(){
        if(Robot.isSimulation())
            return simMeters;
        return 0.0; //TODO: actual claw encoder position
    }

    void extendPneumatic(){
        m_solenoid.set(true);
    }
    void retractPneumatic(){
        m_solenoid.set( false);
    }

    void extendClaw(){
        setpointMeters = 0;
        climbMaster.getPIDController().setReference(0, ControlType.kSmartMotion);
        climbFollower.getPIDController().setReference(0,ControlType.kSmartMotion);
        //TODO: make it actually happen
    }

    void retractClaw(){
        setpointMeters = 0.0;
        //todo: admire the windows 11 octopus emoji (hes so cute)
    }

}
