package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateControllerSub;
import static frc.robot.Constants.EFCConstants.*;

public class EFSubsystem extends SubsystemBase {

    StateControllerSub stateControllerSub;
    public EFSubsystem(StateControllerSub stateControllerSub) {
        this.stateControllerSub = stateControllerSub;
    }

    @Override
    public void periodic() {
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

    }


}
