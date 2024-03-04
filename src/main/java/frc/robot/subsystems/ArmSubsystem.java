package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;
import static  frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {

//DutyCycleEncoder armDutyCycle = new DutyCycleEncoder(5);

    AbsoluteEncoder encoder;
    StateControllerSub stateControllerSub;

    double setpointRad = 0.0;
    double simRad = 0.0;

    CANSparkMax armMaster = new CANSparkMax(armMasterID, CANSparkMax.MotorType.kBrushless);


    public ArmSubsystem(StateControllerSub stateControllerSub) {
      //  armDutyCycle.setDistancePerRotation(Math.PI);


        this.stateControllerSub = stateControllerSub;

        armMaster.restoreFactoryDefaults();


        armMaster.setSmartCurrentLimit(30);
      //  armMaster.setSecondaryCurrentLimit(60, 10);

        encoder = armMaster.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        //   encoder.setAverageDepth(128);

        encoder.setPositionConversionFactor(Math.PI);
        encoder.setVelocityConversionFactor(Math.PI/60);

        //encoder.setPosition(Units.degreesToRadians(5));


        // encoder.
        //  encoder.setInverted(false);



        armMaster.getPIDController().setFeedbackDevice(encoder);


        armMaster.getPIDController().setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0);


      //  armMaster.getEncoder().setPositionConversionFactor(1.0/armGearRatio * 2*Math.PI);
        //armMaster.getEncoder().setPosition(Units.degreesToRadians(14));


        armMaster.setIdleMode(CANSparkBase.IdleMode.kCoast);

        //armMaster.getEncoder().setVelocityConversionFactor(1.0/armGearRatio*2*Math.PI/60);//rad/sec

        armMaster.getPIDController().setP(0.03,0);
       // armMaster.getPIDController().setI(0.1,0);


   //     armMaster.getPIDController().setD(0.1,0);
      //  armMaster.getPIDController().setI(0.01,0);

        armMaster.getPIDController().setSmartMotionMaxAccel(1.5, 0);
        armMaster.getPIDController().setSmartMotionMaxVelocity(2,0);


    //    armMaster.getPIDController().setFeedbackDevice(armMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature,8192));


        armMaster.getPIDController().setOutputRange(0,0);

        //armMaster.setSmartCurrentLimit(30);

 //       SmartDashboard.putNumber("armReference",0);



        armMaster.burnFlash();

        SmartDashboard.putNumber("armTuningP",0.03);
        SmartDashboard.putNumber("armTuningI",0);
        SmartDashboard.putNumber("armTuningD",0);

    }

    double lastP = 0.03;
    double lastI = 0;
    double lastD = 0;

    double lastZeroingTimestamp = 0;


    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simRad += (setpointRad - simRad) * 0.1; //TODO: move to simulation periodic?

        if(stateControllerSub.getDuckMode() == StateControllerSub.DuckMode.UNDUCK && stateControllerSub.getArmState()!= StateControllerSub.ArmState.TRAP
        ){

                setAngleRad(holdingRadSafe);
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

     //   SmartDashboard.putNumber("armAbsDutyCycle", armDutyCycle.getAbsolutePosition());
     //   SmartDashboard.putNumber("armAbsDutyCycleDeg",Units.radiansToDegrees(getDutyCycleRad()));


   //     if(Timer.getFPGATimestamp() - lastZeroingTimestamp > 10){
   //         armMaster.getEncoder().setPosition(getDutyCycleRad());
   //         lastZeroingTimestamp = Timer.getFPGATimestamp();
   //     }


        SmartDashboard.putNumber("getAngleDeg",Units.radiansToDegrees(getArmAngle()));
      //  SmartDashboard.putNumber("armError",)
   //     SmartDashboard.putNumber("absoluteThroughSPARK",encoder.getPosition());

        SmartDashboard.putNumber("rawArmEncoder",encoder.getPosition());

        SmartDashboard.putNumber("armError",getArmAngle() - setpointRad);


        double currentP = SmartDashboard.getNumber("armTuningP",0);
        double currentI = SmartDashboard.getNumber("armTuningI",0);
        double currentD = SmartDashboard.getNumber("armTuningD",0);
        if(currentP!=lastP || currentI!=lastI || currentD !=lastD){
            lastP = currentP;
            lastI = currentI;
            lastD = currentD;

            armMaster.getPIDController().setP(currentP);
            armMaster.getPIDController().setI(currentI);
            armMaster.getPIDController().setD(currentD);

        }
    }

    public double getDutyCycleRad(){
     //   double out = armDutyCycle.getAbsolutePosition();
        double out = 0;

        double zeroReading = 0.226;
        double zeroAngleHalfRots = Units.degreesToRotations(97) * 2;

        out -=zeroReading;
        out +=zeroAngleHalfRots;

        out%=1;

        out*=Math.PI;

        if(out>Units.degreesToRadians(150)) out-=Math.PI;
        return out;
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
        return encoder.getPosition(); //TODO: actual arm angle

    }

}

