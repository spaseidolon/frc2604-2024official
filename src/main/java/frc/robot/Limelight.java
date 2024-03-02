package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.math.Vector2;

public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry camMode = table.getEntry("camMode");
    private static NetworkTableEntry ledMode = table.getEntry("ledMode");
   // private static NetworkTableEntry Pipeline = table.getEntry("pipeline");
    private static NetworkTableEntry streaml = table.getEntry("stream");
    public NetworkTableEntry horizontalOffset = table.getEntry("tx");

    
    public static void enableTracking() {
        //rapidly switching CAM Mode causes Limelight to crash, so pick one and keep it
        camMode.setNumber(0);//tracking vision
        ledMode.setNumber(3);//LED on
        streaml.setNumber(1);
    }

    public static void disableTracking() {
        camMode.setNumber(1);//regular camera
        ledMode.setNumber(1);//LED off
        streaml.setNumber(1);
    }

    public static boolean hasTarget() {
        return table.getEntry("tv").getBoolean(true);
    }

    public static Vector2 getTargetAngle() {
        return new Vector2(table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue());
    }

    public double getHorizontalOffset(){
        return horizontalOffset.getDouble(0.0);
    }

    public double rotatetoTarget(double PID){
        //Calculates power necessary to shift drivetrain and align with the target
        double power = horizontalOffset.getDouble(0)*PID;
        return power;
    }
}