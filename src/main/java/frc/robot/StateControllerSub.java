package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MBUtils;

import java.util.Optional;

import static frc.robot.Constants.ShootingTables;

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
    private double armAngleRad = 0.0;

    public ArmState getArmState(){
        return armState;
    }
    public EFState getEFState(){
        return efState;
    }
    public ClimbState getClimbState(){
        return climbState;
    }

    private double distanceToMySpeaker(){
        Translation2d speakerTranslation = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
        return Math.hypot(speakerTranslation.getX()-robotPose.getX(),speakerTranslation.getY()-robotPose.getY());
    }

    public double getSpeakerAngle(){ //TODO: arm angle in radians
        if(SmartDashboard.getBoolean("tuningMode",true))
            return Units.degreesToRadians(SmartDashboard.getNumber("tuningAngle",90));


        Optional<Double> angle = MBUtils.interpolate(ShootingTables.dist,ShootingTables.angle, distanceToMySpeaker());
        if(angle.isPresent())
            return Units.degreesToRadians(angle.get());

        return Units.degreesToRadians(90);
    }
    public double getSpeakerFlywheelVel(){ //TODO: flywheel velocity in radians per second
        if(SmartDashboard.getBoolean("tuningMode",true))
            return SmartDashboard.getNumber("tuningFlywheelVel",10);

        Optional<Double> vel = MBUtils.interpolate(ShootingTables.dist,ShootingTables.velocity, distanceToMySpeaker());
        if(vel.isPresent())
            return Units.degreesToRadians(vel.get());

        return Units.degreesToRadians(10);
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


    public StateControllerSub(){
        NamedCommands.registerCommand("intakeMode",new InstantCommand(this::intakePressed));
        NamedCommands.registerCommand("holdMode",new InstantCommand(this::holdPressed));
        NamedCommands.registerCommand("setObjectiveSpeaker",new InstantCommand(this::speakerPressed));
        NamedCommands.registerCommand("setObjectiveAmp",new InstantCommand(this::ampPressed));
        NamedCommands.registerCommand("readyToShootMode",new InstantCommand(this::readyToShootPressed));
        NamedCommands.registerCommand("ejectMode",new InstantCommand(this::ejectPressed));

        SmartDashboard.putBoolean("tuningMode",false);
        SmartDashboard.putNumber("tuningAngle",90);
        SmartDashboard.putNumber("tuningFlywheelVel",10);



    }

    double climbAngle = 0.0;


    @Override
    public void periodic(){

        publishTableEntries();

        //get an x y and z from smartDashboard

        if(climbState == ClimbState.STOWED)
            climbAngle += (Units.degreesToRadians(-48) - climbAngle) * 0.1;
        else
            climbAngle += (Units.degreesToRadians(0) - climbAngle) * 0.1;



        Pose3d armPose = new Pose3d(0.24,0,0.21, new Rotation3d(0,armAngleRad-(Math.PI/2),0));
        Pose3d climbPose = new Pose3d(-0.07,0,0.14, new Rotation3d(0,climbAngle,0));

        SmartDashboard.putNumberArray("armPose2D", new double[]{armPose.getX(),armPose.getY(),armPose.getZ(),armPose.getRotation().getQuaternion().getW(),armPose.getRotation().getQuaternion().getX(),armPose.getRotation().getQuaternion().getY(),armPose.getRotation().getQuaternion().getZ()});
        SmartDashboard.putNumberArray("climbPose2D", new double[]{climbPose.getX(),climbPose.getY(),climbPose.getZ(),climbPose.getRotation().getQuaternion().getW(),climbPose.getRotation().getQuaternion().getX(),climbPose.getRotation().getQuaternion().getY(),climbPose.getRotation().getQuaternion().getZ()});


        SmartDashboard.putNumber("distanceToMySpeaker",distanceToMySpeaker());
      //  SmartDashboard.putNumber("robotX",robotPose.getX());
    }

    public void publishTableEntries(){
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

    public void setArmAngleRad(double angle){
        armAngleRad = angle;
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
