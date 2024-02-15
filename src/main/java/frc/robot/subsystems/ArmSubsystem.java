package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;
import static  frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
    StateControllerSub stateControllerSub;

    double setpointRad = 0.0;
    double simRad = 0.0;

    CANSparkMax armMaster = new CANSparkMax(armMasterID, CANSparkMax.MotorType.kBrushless);


    public ArmSubsystem(StateControllerSub stateControllerSub) {

        this.stateControllerSub = stateControllerSub;

        armMaster.restoreFactoryDefaults();


        armMaster.setSmartCurrentLimit(30);
      //  armMaster.setSecondaryCurrentLimit(60, 10);

        armMaster.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kSCurve, 0);

        armMaster.getEncoder().setPositionConversionFactor(1.0/armGearRatio * 2*Math.PI);
        armMaster.getEncoder().setPosition(Units.degreesToRadians(5));


        armMaster.setIdleMode(CANSparkBase.IdleMode.kCoast);

        armMaster.getEncoder().setVelocityConversionFactor(1.0/armGearRatio*2*Math.PI/60);//rad/sec

        armMaster.getPIDController().setP(0.4,0);

        armMaster.getPIDController().setSmartMotionMaxAccel(0.3, 0);
        armMaster.getPIDController().setSmartMotionMaxVelocity(1, 0);




        armMaster.getPIDController().setOutputRange(-0.5,0.5);

 //       SmartDashboard.putNumber("armReference",0);


        armMaster.burnFlash();

    }



    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simRad += (setpointRad - simRad) * 0.1; //TODO: move to simulation periodic?

        if(stateControllerSub.getDuckMode() == StateControllerSub.DuckMode.DUCKING){
            if(stateControllerSub.getArmState() == StateControllerSub.ArmState.INTAKE)
                setAngleRad(intakeRad); //ugh, spaghetti
            else
                setAngleRad(duckingRad);
        }

        else{
            switch (stateControllerSub.getArmState()) {
                case HOLD -> setAngleRad(stateControllerSub.getHoldAngle());
                case SPEAKER -> setAngleRad(stateControllerSub.getSpeakerAngle());
                case AMP -> setAngleRad(ampRad);
                case TRAP -> setAngleRad(stateControllerSub.getTrapArmAngle());
                case INTAKE -> setAngleRad(intakeRad);

            }
        }


        NetworkTableInstance.getDefault().getTable("StateController").getEntry("armAngle").setDouble(getArmAngle());

        stateControllerSub.setArmAngleRad(getArmAngle());


    }

    private void setAngleRad(double angle){
        setpointRad = angle;
        //TODO: actually command a position lol
     //   setpointRad = Units.degreesToRadians(SmartDashboard.getNumber("tuningAngle",90));
       // SmartDashboard.putNumber("measuredArmAngle",Units.radiansToDegrees(getArmAngle()));

       // armMaster.getPIDController().setReference(Units.degreesToRadians(SmartDashboard.getNumber("armReference",0)), CANSparkBase.ControlType.kSmartMotion);

        armMaster.getPIDController().setReference(setpointRad,CANSparkBase.ControlType.kSmartMotion);

    }

    public double getArmAngle(){
        if(Robot.isSimulation())
            return simRad;
        return armMaster.getEncoder().getPosition(); //TODO: actual arm angle

    }

}

