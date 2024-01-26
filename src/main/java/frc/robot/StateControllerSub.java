package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class StateControllerSub {
    public enum ArmState{HOLD,SPEAKER,AMP,TRAP,INTAKE}
    public enum EFState{INTAKE,HOLD,EJECT,SHOOT}
    public enum ClimbState{STOWED,READY,CLIMBED}

    private ArmState armState = ArmState.HOLD;
    private EFState efState = EFState.HOLD;
    private ClimbState climbState = ClimbState.STOWED;

    public ArmState getArmState(){
        return armState;
    }
    public EFState getEFState(){
        return efState;
    }
    public ClimbState getClimbState(){
        return climbState;
    }

    public double getSpeakerAngle(){ //TODO: arm angle in radians
        return 0.0;
    }
    public double getSpeakerFlywheelVel(){ //TODO: flywheel velocity in radians per second
        return 0.0;
    }

    public double getTrapArmAngle(){//TODO: arm angle in radians during trap - maybe this can be set into this sub by a lerp command or something?
        //could you have a sub-path as you come out from under the stage, where its completion is passed into t?
        return 0.0;
    }



}
