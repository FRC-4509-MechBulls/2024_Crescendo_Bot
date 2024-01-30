package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;
import static  frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
    StateControllerSub stateControllerSub;
    public ArmSubsystem(StateControllerSub stateControllerSub) {
        this.stateControllerSub = stateControllerSub;
    }
    double setpointRad = 0.0;
    double simRad = 0.0;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simRad += (setpointRad - simRad) * 0.1;


        switch (stateControllerSub.getArmState()) {
            case HOLD -> setAngleRad(holdingRad);
            case SPEAKER -> setAngleRad(stateControllerSub.getSpeakerAngle());
            case AMP -> setAngleRad(ampRad);
            case TRAP -> setAngleRad(stateControllerSub.getTrapArmAngle());
            case INTAKE -> {
//                if(stateControllerSub.getObjective() == StateControllerSub.Objective.SOURCE)
//                    setAngleRad(sourceRad);
//                else
                    setAngleRad(intakeRad);
            }
        }

        NetworkTableInstance.getDefault().getTable("StateController").getEntry("armAngle").setDouble(getArmAngle());

    }

    private void setAngleRad(double angle){
        setpointRad = angle;
        //TODO: actually command a position lol
    }

    public double getArmAngle(){
        if(Robot.isSimulation())
            return simRad;
        return 0.0; //TODO: actual arm angle

    }

}

