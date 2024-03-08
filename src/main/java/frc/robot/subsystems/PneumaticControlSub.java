package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateControllerSub;

public class PneumaticControlSub extends SubsystemBase {
    PneumaticsControlModule pcm = new PneumaticsControlModule(59);

    boolean compressorEnabledOld = false;
    Solenoid climbSolenoid;
    Solenoid brakeSolenoid;


    StateControllerSub stateControllerSub;

    public PneumaticControlSub(StateControllerSub stateControllerSub){
        pcm.disableCompressor();
        SmartDashboard.putBoolean("enableCompressor",false);

        climbSolenoid = pcm.makeSolenoid(6);
        brakeSolenoid = pcm.makeSolenoid(5);

       SmartDashboard.putBoolean("enableBrakes",true);
this.stateControllerSub = stateControllerSub;

    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("compressorCurrent",pcm.getCompressorCurrent());

        if(compressorEnabled() != compressorEnabledOld){
            compressorEnabledOld = compressorEnabled();
            if(compressorEnabledOld)
                pcm.enableCompressorDigital();
            else
                pcm.disableCompressor();
        }

       // brakeSolenoid.set(SmartDashboard.getBoolean("setBrakeSolenoid",false));



    }

    public boolean compressorEnabled(){
        boolean enableCompressorNT = SmartDashboard.getBoolean("enableCompressor",false);

        return enableCompressorNT || stateControllerSub.getArmState() == StateControllerSub.ArmState.INTAKE;
    }

    public void setClimbSolenoid(boolean state){
        climbSolenoid.set(state);
    }

    public void setBrakeSolenoid(boolean state){
        SmartDashboard.putBoolean("brakeSolenoidClosed",state);
        if(!SmartDashboard.getBoolean("enableBrakes",true))
            state = false;
        stateControllerSub.feedBrakeEngaged(state);
        brakeSolenoid.set(state);
    }


}
