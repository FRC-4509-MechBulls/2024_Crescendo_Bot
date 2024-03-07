package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticControlSub extends SubsystemBase {
    PneumaticsControlModule pcm = new PneumaticsControlModule(59);

    boolean enableCompressor = false;
    Solenoid climbSolenoid;
    Solenoid brakeSolenoid;

    public PneumaticControlSub(){
        pcm.disableCompressor();
        SmartDashboard.putBoolean("enableCompressor",false);

        climbSolenoid = pcm.makeSolenoid(6);
        brakeSolenoid = pcm.makeSolenoid(5);

       SmartDashboard.putBoolean("enableBrakes",true);


    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("compressorCurrent",pcm.getCompressorCurrent());

        boolean enableCompressorNT = SmartDashboard.getBoolean("enableCompressor",false);
        if(enableCompressorNT != enableCompressor){
            enableCompressor = enableCompressorNT;
            if(enableCompressor)
                pcm.enableCompressorDigital();
            else
                pcm.disableCompressor();
        }

       // brakeSolenoid.set(SmartDashboard.getBoolean("setBrakeSolenoid",false));



    }

    public void setClimbSolenoid(boolean state){
        climbSolenoid.set(state);
    }

    public void setBrakeSolenoid(boolean state){
        SmartDashboard.putBoolean("brakeSolenoidClosed",state);
        if(!SmartDashboard.getBoolean("enableBrakes",true))
            state = false;
        brakeSolenoid.set(state);
    }


}
