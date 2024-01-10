package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class FMSGetter {

    static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    static NetworkTable table = inst.getTable("FMSInfo");

    public static boolean isRedAlliance(){
        if(table.getValue("IsRedAlliance").isBoolean())
            return table.getValue("IsRedAlliance").getBoolean();
        return false;
    }

}
