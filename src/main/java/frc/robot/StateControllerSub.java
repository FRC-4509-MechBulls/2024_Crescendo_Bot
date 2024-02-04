package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateControllerSub extends SubsystemBase {



    private NetworkTable table = NetworkTableInstance.getDefault().getTable("StateController");

    public enum ArmState{HOLD,SPEAKER,AMP,TRAP,INTAKE,SOURCE}
    public enum EFState{HOLD,INTAKE,EJECT,READY,SHOOT}
    public enum ClimbState{STOWED,READY,CLIMBED}
    public enum Objective{SPEAKER,AMP,SOURCE,TRAP}
    public enum SelectedTrap{AMP,SOURCE,REAR}

    private ArmState armState = ArmState.HOLD;
    private EFState efState = EFState.HOLD;
    private ClimbState climbState = ClimbState.STOWED;
    Objective objective = Objective.SPEAKER;
    SelectedTrap selectedTrap = SelectedTrap.AMP;
    private Pose2d robotPose = new Pose2d();

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

    public double getTrapArmAngle(){
        //TODO: arm angle in radians during trap - maybe this can be set into this sub by a lerp command or something?
        //could you have a sub-path as you come out from under the stage, where its completion is passed into t?
        return 0.0;
    }

    public Pose2d getRobotPose(){
        return robotPose;
    }

    public void feedRobotPose(Pose2d pose){
        robotPose = pose;
    }

    public void setArmState(ArmState state){
        armState = state;
    }

    public Objective getObjective(){return objective;}


    @Override
    public void periodic(){

        table.getEntry("odom_x").setDouble(robotPose.getX());
        table.getEntry("odom_y").setDouble(robotPose.getY());
        table.getEntry("odom_rad").setDouble(robotPose.getRotation().getRadians());

        table.getEntry("armState").setString(armState.toString());
        table.getEntry("efState").setString(efState.toString());
        table.getEntry("climbState").setString(climbState.toString());
        table.getEntry("objective").setString(objective.toString());
        table.getEntry("selectedTrap").setString(selectedTrap.toString());


    }

    public void intakePressed(){

        armState = ArmState.INTAKE;
        efState = EFState.INTAKE;
       // climbState = ClimbState.STOWED; //TODO: should this be here?
    }
    public void holdPressed(){
        armState = ArmState.HOLD;
        efState = EFState.HOLD;
    }
    public void ejectPressed(){
        //armState = ArmState.HOLD;
        //efState = EFState.EJECT;
    }
    public void readyToShootPressed(){
       // armState = ArmState.HOLD;
        efState = EFState.READY;
        switch (objective) {
            case AMP -> armState = ArmState.AMP;
            case SPEAKER -> armState = ArmState.SPEAKER;
            case TRAP -> armState = ArmState.TRAP;
            case SOURCE -> armState = ArmState.SOURCE;
        }

      //  efState = EFState.READY;
    }

    public void raiseClimbPressed(){
        climbState = ClimbState.READY;
    }
    public void climbPressed(){
        climbState = ClimbState.CLIMBED;
    }
    public void stowPressed(){
        climbState = ClimbState.STOWED;
    }

    public void speakerPressed(){
        objective = Objective.SPEAKER;
       // efState = EFState.HOLD;
    }
    public void ampPressed(){
        objective = Objective.AMP;
       // efState = EFState.HOLD;
    }
    public void trapPressed(){
        objective = Objective.TRAP;
       // efState = EFState.HOLD;
    }

    public void sourcePressed(){
        objective = Objective.SOURCE;
       // efState = EFState.INTAKE;
    }

    public void shootPressed(){
        readyToShootPressed();
        efState = EFState.SHOOT;
    }







}
