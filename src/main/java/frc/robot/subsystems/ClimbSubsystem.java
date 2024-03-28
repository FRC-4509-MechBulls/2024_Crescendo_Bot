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

PneumaticControlSub pneumaticControlSub;


    public ClimbSubsystem(StateControllerSub stateControllerSub,PneumaticControlSub pneumaticControlSub) {




//pcm.makeDoubleSolenoid(0,0).set();


        this.stateControllerSub = stateControllerSub;



        this.pneumaticControlSub = pneumaticControlSub;
    }



    double setpointMeters = 0.0;//bottom
    double simMeters = 0.0;




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(Robot.isSimulation())
            simMeters += (setpointMeters - simMeters) * 0.1;

        switch (stateControllerSub.getClimbStateConsideringDuckMode()){
            case READY:
                extendClaw();
                break;
            case CLIMBED:
                retractClaw();
                break;
        }

    }



    void extendClaw(){
pneumaticControlSub.setClimbSolenoid(true);
        //TODO: make it actually happen
    }

    void retractClaw(){
        pneumaticControlSub.setClimbSolenoid(false);
    }

}
