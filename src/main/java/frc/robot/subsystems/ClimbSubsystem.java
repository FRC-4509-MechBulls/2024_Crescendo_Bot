package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.StateControllerSub;

public class ClimbSubsystem extends SubsystemBase {
StateControllerSub stateControllerSub;
    public ClimbSubsystem(StateControllerSub stateControllerSub) {
        this.stateControllerSub = stateControllerSub;
    }

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
        //TODO: make arm go to stowed position
    }
    void retractPneumatic(){
        //TODO: make arm go to ready position
    }

    void extendClaw(){
        setpointMeters = maxClawDistanceMeters;
        //TODO: make it actually happen
    }

    void retractClaw(){
        setpointMeters = 0.0;
        //todo: admire the windows 11 octopus emoji (hes so cute)
    }

}
