package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
StateControllerSub stateControllerSub;

    CANSparkMax climbPrimary = new CANSparkMax(climbMasterID, CANSparkMax.MotorType.kBrushless);
    CANSparkMax climbSecondary = new CANSparkMax(climbFollowerID, CANSparkMax.MotorType.kBrushless);



    public ClimbSubsystem(StateControllerSub stateControllerSub) {

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

        climbPrimary.getPIDController().setP(0.4);
        climbSecondary.getPIDController().setP(0.4);

        climbPrimary.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);
        climbSecondary.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal,0);

        climbPrimary.getPIDController().setSmartMotionMaxVelocity(0.5,0);
        climbSecondary.getPIDController().setSmartMotionMaxVelocity(0.5,0);

        climbPrimary.getPIDController().setSmartMotionMaxAccel(0.5,0);
        climbSecondary.getPIDController().setSmartMotionMaxAccel(0.5,0);

        climbPrimary.getPIDController().setOutputRange(-0.4,0.4);
        climbSecondary.getPIDController().setOutputRange(-0.4,0.4);


        climbPrimary.burnFlash();
        climbSecondary.burnFlash();
    }

    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);


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
    }

    public double getClawPosition(){
        if(Robot.isSimulation())
            return simMeters;
        return climbPrimary.getEncoder().getPosition(); //TODO: actual claw encoder position
    }

    void extendPneumatic(){
        solenoid.set(true);
    }
    void retractPneumatic(){
        solenoid.set( false);
    }

    void extendClaw(){
        setpointMeters = 0;
        climbPrimary.getPIDController().setReference(0, ControlType.kSmartMotion);
        climbSecondary.getPIDController().setReference(0,ControlType.kSmartMotion);
        SmartDashboard.putBoolean("clawExtended",true);
        //TODO: make it actually happen
    }

    void retractClaw(){
        setpointMeters = 1.0;

        climbPrimary.getPIDController().setReference(-1, ControlType.kSmartMotion);
        climbSecondary.getPIDController().setReference(1,ControlType.kSmartMotion);
        SmartDashboard.putBoolean("clawExtended",false);

        //todo: admire the windows 11 octopus emoji (hes so cute)
    }

}
