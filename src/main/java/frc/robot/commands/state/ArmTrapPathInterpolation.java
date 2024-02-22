package frc.robot.commands.state;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateControllerSub;
import frc.robot.util.MBUtils;

public class ArmTrapPathInterpolation extends Command {

    private StateControllerSub stateControllerSub;
    private PathPlannerTrajectory pathPlannerTrajectory;
    private double armAngleStart;
    private double armAngleFinal;
    private Timer runtime = new Timer();

    public ArmTrapPathInterpolation(StateControllerSub stateControllerSub, PathPlannerTrajectory pathPlannerTrajectory,double armAngleStart,double armAngleFinal){
        this.stateControllerSub = stateControllerSub;
        this.pathPlannerTrajectory = pathPlannerTrajectory;
        this.armAngleStart = armAngleStart;
        this.armAngleFinal = armAngleFinal;
    }


    @Override
    public void initialize(){
        runtime.start();
    }

    @Override
    public void execute(){
        double percentCompleted = runtime.get() / pathPlannerTrajectory.getTotalTimeSeconds(); //TODO: find a better way that accounts for accel constraints?
        double armAngle = MBUtils.lerp(armAngleStart,armAngleFinal,percentCompleted);
        stateControllerSub.feedTrapArmAngle(armAngle);
    }

    @Override
    public boolean isFinished(){
        return runtime.get() >= pathPlannerTrajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted){

    }





}
