package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateControllerSub;

public class ArmSubsystem extends SubsystemBase {
    StateControllerSub stateControllerSub;
    public ArmSubsystem(StateControllerSub stateControllerSub) {
        this.stateControllerSub = stateControllerSub;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}

