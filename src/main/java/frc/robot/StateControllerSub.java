package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.Alignments;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MBUtils;

import java.util.Optional;

import static frc.robot.Constants.ShootingTables;

public class StateControllerSub extends SubsystemBase {



    private NetworkTable table = NetworkTableInstance.getDefault().getTable("StateController");

    public enum ArmState{HOLD,SPEAKER,AMP,TRAP,INTAKE,SOURCE}
    public enum EFState{HOLD,INTAKE,EJECT,READY,SHOOT}
    public enum ClimbState{DOWN,READY,CLIMBED} //only ever set to ready or climbed
    public enum Objective{SPEAKER,AMP,SOURCE,TRAP}
    public enum SelectedTrap{AMP,SOURCE,REAR}

    public enum UseFedPoseIntention{YES,NO}
    boolean aimAssistPPEnabled = false;

    public enum DuckMode {UNDUCK, DOWN}

    private ArmState armState = ArmState.HOLD;
    private EFState efState = EFState.HOLD;
    private ClimbState climbState = ClimbState.DOWN;
    Objective objective = Objective.SPEAKER;
    SelectedTrap selectedTrap = SelectedTrap.AMP;

    UseFedPoseIntention useFedPoseIntention = UseFedPoseIntention.NO;

    Pose2d intendedPose = new Pose2d();

    DuckMode duckMode = DuckMode.DOWN;

    boolean alignWhenCloseEnabled = true;


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

    public Optional<Rotation2d> getRotationTargetOverride(){
        if(objective == Objective.SOURCE && distanceToObjective(Objective.SOURCE) > 3)
            return Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(0)));

        if(objective == Objective.AMP && distanceToObjective(Objective.AMP) > 3)
            return Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(180)));


        if(armState ==ArmState.INTAKE && aimAssistPPEnabled)
            return Optional.of(getRobotPose().getRotation().minus(Rotation2d.fromRadians(noteAlignAngleDiff)));

        if(armState ==ArmState.SPEAKER  && aimAssistPPEnabled)
            return Optional.of(getRobotPose().getRotation().minus(Rotation2d.fromRadians(MBUtils.angleDiffRad(angleToObjective(objective),robotPose.getRotation().getRadians()))));



        return Optional.empty();
    }

    public void setPPAimAssistEnabled(boolean intakeAssistPPEnabled){
        this.aimAssistPPEnabled = intakeAssistPPEnabled;
    }


    public void scheduleAlignmentCommand(){
        setDuckMode(true);
        armState = ArmState.HOLD;



        if(objective == Objective.AMP)
            Alignments.ampAlign.schedule();
        if(objective == Objective.SOURCE)
            Alignments.sourceAlign().schedule();
        if(objective == Objective.SPEAKER)
            Alignments.speakerAlign().schedule();

    }



    public DuckMode getDuckMode(){
        return duckMode;
    }



    private double distanceToObjective(Objective objective){
            Translation2d objectiveTranslation = positionOfObjective(objective);
        return Math.hypot(objectiveTranslation.getX()-robotPose.getX(),objectiveTranslation.getY()-robotPose.getY());
        }

        private Translation2d positionOfObjective(Objective objective) {
            Translation2d objectiveTranslation = switch (objective) {
                case AMP -> FieldConstants.ampCenter;
                case SPEAKER -> FieldConstants.Speaker.centerSpeakerOpening.getTranslation();
                case TRAP -> new Translation2d(FieldConstants.podiumX, FieldConstants.fieldWidth / 2.0);
                case SOURCE -> FieldConstants.sourceCenterRough;
            };
            return AllianceFlipUtil.apply(objectiveTranslation);
        }

        private double angleToObjective(Objective objective){
            Translation2d objectiveTranslation = positionOfObjective(objective);
            return Math.atan2(objectiveTranslation.getY()-robotPose.getY(),objectiveTranslation.getX()-robotPose.getX());
        }


    private double distanceToMySpeaker(Pose2d robotPose){
        Translation2d speakerTranslation = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());
        return Math.hypot(speakerTranslation.getX()-robotPose.getX(),speakerTranslation.getY()-robotPose.getY());
    }
    private double distanceToMySpeaker(){
        return distanceToMySpeaker(robotPose);
    }

    public double getHoldAngle(){ // this is redundant now :)
//        if(duckMode == DuckMode.DUCKING)
//            return Constants.ArmConstants.duckingRad;
        return Constants.ArmConstants.duckingRad;
    }

    public boolean tuningMode(){
        return SmartDashboard.getBoolean("tuningMode",false);
    }

    public void setUseFedPoseIntention(UseFedPoseIntention useFedPoseIntention){
        this.useFedPoseIntention = useFedPoseIntention;
    }
    public UseFedPoseIntention getUseFedPoseIntention(){
        return useFedPoseIntention;
    }
    public void feedIntendedPose(Pose2d pose){
        intendedPose = pose;
        SmartDashboard.putNumberArray("intendedShotPose",new double[]{pose.getX(),pose.getY(),pose.getRotation().getRadians()});
    }

    public void feedIntendedPoseWithAllianceFlip(Pose2d pose){
        feedIntendedPose(AllianceFlipUtil.apply(pose));
    }


    public double getSpeakerAngle(){ //TODO: arm angle in radians
        if(tuningMode())
            return Units.degreesToRadians(SmartDashboard.getNumber("tuningAngle",90));

     //   if(useFedPoseIntention == UseFedPoseIntention.YES)
        double distanceToUse = distanceToMySpeaker();

        if(useFedPoseIntention == UseFedPoseIntention.YES)
            distanceToUse = distanceToMySpeaker(intendedPose);

        Optional<Double> angle = MBUtils.interpolateOrExtrapolateFlat(ShootingTables.dist,ShootingTables.angle, distanceToUse);
        if(angle.isPresent())
            return Units.degreesToRadians(angle.get());

        return Units.degreesToRadians(90);
    }
    public double getSpeakerFlywheelVel(){ //TODO: flywheel velocity in radians per second
        if(tuningMode())
            return SmartDashboard.getNumber("tuningFlywheelVel",10);
        double distanceToUse = distanceToMySpeaker();

        if(useFedPoseIntention == UseFedPoseIntention.YES)
            distanceToUse = distanceToMySpeaker(intendedPose);

        Optional<Double> vel = MBUtils.interpolateOrExtrapolateFlat(ShootingTables.dist,ShootingTables.velocity, distanceToUse);
        if(vel.isPresent())
            return vel.get();

        return 10;
    }

    private double fedTrapArmAngle = Constants.ArmConstants.intakeRad;

    public double getTrapArmAngle(){
        //TODO: arm angle in radians during trap - maybe this can be set into this sub by a lerp command or something?
        //could you have a sub-path as you come out from under the stage, where its completion is passed into t?
        return fedTrapArmAngle;
    }

    public void feedTrapArmAngle(double fedTrapArmAngle){
        this.fedTrapArmAngle = fedTrapArmAngle;
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

public ClimbState getClimbStateConsideringDuckMode(){
    if(duckMode==DuckMode.UNDUCK)
        return ClimbState.CLIMBED;
    return climbState;
}



public void toggleClimbed(){
        if(climbState == ClimbState.DOWN) //local variable climbState should never be down. (that's what the climbStateConsideringDuckMode() method is for)
            climbState = ClimbState.CLIMBED;

        if(climbState == ClimbState.CLIMBED)
            climbState = ClimbState.READY;
        else if(climbState == ClimbState.READY)
            climbState = ClimbState.CLIMBED;
}

public void setClimbState(ClimbState climbState){
        this.climbState = climbState;
}
    DigitalInput beamBreak1 = new DigitalInput(4);


    CommandXboxController driver;
    CommandXboxController operator;
    public StateControllerSub(CommandXboxController driver, CommandXboxController operator){
        NamedCommands.registerCommand("intakeMode",new InstantCommand(this::intakePressed));
        NamedCommands.registerCommand("holdMode",new InstantCommand(this::holdPressed));
        NamedCommands.registerCommand("setObjectiveSpeaker",new InstantCommand(this::speakerPressed));
        NamedCommands.registerCommand("setObjectiveAmp",new InstantCommand(this::ampPressed));
        NamedCommands.registerCommand("readyToShootMode",new InstantCommand(this::readyToShootPressed));
        NamedCommands.registerCommand("shootMode",new InstantCommand(this::shootPressed));

        NamedCommands.registerCommand("enableAimAssist",new InstantCommand(()-> setPPAimAssistEnabled(true)));
        NamedCommands.registerCommand("disableAimAssist",new InstantCommand(()-> setPPAimAssistEnabled(false)));

        NamedCommands.registerCommand("enableUseFedPoseIntention",new InstantCommand(()->setUseFedPoseIntention(UseFedPoseIntention.YES)));
        NamedCommands.registerCommand("disableUseFedPoseIntention",new InstantCommand(()->setUseFedPoseIntention(UseFedPoseIntention.NO)));

        NamedCommands.registerCommand("shotIntentionCurrentPose",new InstantCommand(()->feedIntendedPose(robotPose)));

        NamedCommands.registerCommand("shotIntentionAgainstSpeaker",new InstantCommand(()->feedIntendedPoseWithAllianceFlip(FieldConstants.Speaker.centerSpeakerOpening)));


        SmartDashboard.putBoolean("tuningMode",false);
        SmartDashboard.putNumber("tuningAngle",90);
        SmartDashboard.putNumber("tuningFlywheelVel",10);

        SmartDashboard.putNumber("groundHeight",0);

        SmartDashboard.putBoolean("alignWhenClose",alignWhenCloseEnabled);


        this.driver = driver;
        this.operator = operator;

    }

    double climbAngle = 0.0;

    public void feedClimbAngle(double climbAngle){
        this.climbAngle = climbAngle;
    }


    @Override
    public void periodic(){

        publishTableEntries();

        updateRumbles();

        //get an x y and z from smartDashboard




        Pose3d armPose = new Pose3d(0.24,0,0.21, new Rotation3d(0,armAngleRad-(Math.PI/2),0));
        Pose3d climbPose = new Pose3d(-0.07,0,0.14, new Rotation3d(0,climbAngle,0));

        //publish arm and climb 3d poses to network tables
        table.getEntry("armPose").setDoubleArray(new double[]{armPose.getX(),armPose.getY(),armPose.getZ(),armPose.getRotation().getQuaternion().getW(),armPose.getRotation().getQuaternion().getX(),armPose.getRotation().getQuaternion().getY(),armPose.getRotation().getQuaternion().getZ()});
        table.getEntry("climbPose").setDoubleArray(new double[]{climbPose.getX(),climbPose.getY(),climbPose.getZ(),climbPose.getRotation().getQuaternion().getW(),climbPose.getRotation().getQuaternion().getX(),climbPose.getRotation().getQuaternion().getY(),climbPose.getRotation().getQuaternion().getZ()});

        double groundHeight = SmartDashboard.getNumber("groundHeight",0);

        Pose3d odometry3D = new Pose3d(robotPose.getX(),robotPose.getY(),groundHeight,new Rotation3d(0,0,robotPose.getRotation().getRadians()));

        table.getEntry("odometry3D").setDoubleArray(new double[]{odometry3D.getX(),odometry3D.getY(),odometry3D.getZ(),odometry3D.getRotation().getQuaternion().getW(),odometry3D.getRotation().getQuaternion().getX(),odometry3D.getRotation().getQuaternion().getY(),odometry3D.getRotation().getQuaternion().getZ()});


        SmartDashboard.putNumber("distanceToMySpeaker",distanceToMySpeaker());
      //  SmartDashboard.putNumber("robotX",robotPose.getX());

        if(efState == EFState.READY || efState == EFState.SHOOT) {
            switch (objective) {
                case AMP -> armState = ArmState.AMP;
                case SPEAKER -> armState = ArmState.SPEAKER;
                case TRAP -> armState = ArmState.TRAP;
                case SOURCE -> armState = ArmState.SOURCE;
            }
        }

        SmartDashboard.putNumber("distToObjective",distanceToObjective(objective));

        if(duckMode == DuckMode.UNDUCK){
            if(climbState == ClimbState.DOWN)
                climbState = ClimbState.READY;
        }else{
               // climbState = ClimbState.DOWN;
        }



        if(makeEFHoldTimer.hasElapsed(0.3)){
            efState = EFState.HOLD;
            makeEFHoldTimer.stop();
            makeEFHoldTimer.reset();

        }

        if(!beamBreak1.get() && armState == ArmState.INTAKE){
           // armState = ArmState.HOLD;
           // efState = EFState.HOLD;
        }

        alignWhenCloseEnabled = SmartDashboard.getBoolean("alignWhenClose",true);

    }

    double lastUnacceptableErrorTime = 0;
    double armError = 0;

    public void updateRumbles(){
        if(Math.abs(armError) > Units.degreesToRadians(1))
            lastUnacceptableErrorTime = Timer.getFPGATimestamp();
        boolean readyToShoot = Timer.getFPGATimestamp() - lastUnacceptableErrorTime > 0.25;


        double bothRumbleVal = 0;
        double operatorRumbleVal = 0;

        if(Timer.getFPGATimestamp() - lastBrakeEngagementTimestamp <0.3 && armState == ArmState.SPEAKER)
            bothRumbleVal = 1;

        if(readyToShoot && armState == ArmState.SPEAKER)
            operatorRumbleVal +=1;

        driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble,bothRumbleVal);
        operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,bothRumbleVal + operatorRumbleVal);

    }
    boolean brakeEngaged = false;
    double lastBrakeEngagementTimestamp = 0;
    public void feedBrakeEngaged(boolean brakeEngaged){
        if(brakeEngaged == true && this.brakeEngaged == false)
            onBrakeEngaged();
        this.brakeEngaged = brakeEngaged;
    }
    public void feedArmError(double armError){this.armError = armError;}

    void onBrakeEngaged(){
        lastBrakeEngagementTimestamp = Timer.getFPGATimestamp();
    }


    public void setDuckMode(boolean down){
        if(down) duckMode = DuckMode.DOWN;
        else     duckMode = DuckMode.UNDUCK;
    }


   public boolean alignWhenClose() {
        if(objective == Objective.SPEAKER && distanceToMySpeaker() < 6)
            return alignWhenCloseEnabled;
        if(armState == ArmState.INTAKE)
            return alignWhenCloseEnabled;
        return false;
    }



    public double alignWhenCloseAngDiff() {
        if(armState == ArmState.INTAKE){
            return MBUtils.clamp(noteAlignAngleDiff,1);
        }

        return MBUtils.angleDiffRad(angleToObjective(objective),robotPose.getRotation().getRadians());
    }
    double noteAlignAngleDiff = 0;

    public void feedNoteAlignAngleDiff(double noteAlignAngleDiff){
        this.noteAlignAngleDiff = noteAlignAngleDiff;
    }



  /*  public void toggleAlignWhenClose() {
        this.alignWhenClose = !this.alignWhenClose;
    }
*/
    public void publishTableEntries(){
       // table.getEntry("odom_x").setDouble(robotPose.getX());
       // table.getEntry("odom_y").setDouble(robotPose.getY());
       // table.getEntry("odom_rad").setDouble(robotPose.getRotation().getRadians());

        table.getEntry("odometry").setDoubleArray(new double[]{robotPose.getX(),robotPose.getY(),robotPose.getRotation().getRadians()});



        table.getEntry("armState").setString(armState.toString());
        table.getEntry("efState").setString(efState.toString());
        table.getEntry("climbState").setString(climbState.toString());
        table.getEntry("objective").setString(objective.toString());
        table.getEntry("selectedTrap").setString(selectedTrap.toString());

        table.getEntry("loweredMode").setString(duckMode.toString());
    }

    public void resetPressed(){
        armState = ArmState.HOLD;
        efState = EFState.HOLD;
        climbState = ClimbState.DOWN;
        useFedPoseIntention = UseFedPoseIntention.NO;

    }

    public void intakePressed(){

        armState = ArmState.INTAKE;
        efState = EFState.INTAKE;
       // climbState = ClimbState.STOWED; //TODO: should this be here?
    }

    public void stowClimbPressed(){
        climbState = ClimbState.DOWN;
    }
    public void raiseClimbPressed(){
        climbState = ClimbState.READY;
    }
    public void climbPressed(){
        climbState = ClimbState.CLIMBED;
    }


    Timer makeEFHoldTimer = new Timer();

    public void holdPressed(){
        armState=ArmState.HOLD;

        if(efState == EFState.INTAKE){
            efState= EFState.EJECT;
            makeEFHoldTimer.reset();
            makeEFHoldTimer.start();
        }else{
            efState = EFState.HOLD;
        }

    }
    public void ejectPressed(){
        //armState = ArmState.HOLD;
        efState= EFState.EJECT;
        makeEFHoldTimer.reset();
        makeEFHoldTimer.start();
    }
    public void readyToShootPressed(){
       // armState = ArmState.HOLD;
        efState = EFState.READY;

      //  efState = EFState.READY;
    }

    public void setArmAngleRad(double angle){
        armAngleRad = angle;
    }

public void setObjective(Objective objective){
        this.objective = objective;
}

public void setEfState(EFState efState){
        this.efState = efState;
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
