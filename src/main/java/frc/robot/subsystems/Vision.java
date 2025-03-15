package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase{

    private String ll_name;
    private NetworkTable nt_table;
    private boolean targetRightCoral;

    private double tx, ty, ta;
    
    private long tid;
    private double[] targetpose_robotspace = new double[6];

    public Vision(String ll_name) {
        this.ll_name = ll_name;
        nt_table = NetworkTableInstance.getDefault().getTable(this.ll_name);
        targetRightCoral = false; //Default target left coral
        // Other reference frame options 
        //private double [] targetpose_cameraspace = fwd_table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        //private double [] botpose_targetspace = fwd_table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("limelight-fwd", 
            LimelightConstants.FWD_OFFSET,    // Forward offset (meters)
            LimelightConstants.SIDE_OFFSET,    // Side offset (meters)
            LimelightConstants.HEIGHT_OFFSET,    // Height offset (meters)
            LimelightConstants.ROLL,    // Roll (degrees)
            LimelightConstants.PITCH,   // Pitch (degrees)
            LimelightConstants.YAW     // Yaw (degrees)
            );
        //Shuffleboard.getTab("Driver Data").addCamera("#[Driver View]", "limelight-fwd", "mjpeg:http://10.88.41.13:5800")
        //.withProperties(Map.of("showControls", "false"));

    }

    public double[] getTargetpose_Robotspace() {
        return targetpose_robotspace;
    }

    public long getTargetedApril() {
        return tid;
    }

    public boolean visionAtSetpoint()
    {
        // TODO return true once near setpoint
        return false;
    }

    public void setTargetCoralToLeft()
    {
        targetRightCoral = false;
    }

    public void setTargetCoralToRight()
    {
        targetRightCoral = true;
    }
    

    @Override
    public void periodic() {
        targetpose_robotspace = nt_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        tid = nt_table.getEntry("tid").getInteger(0); // Primary April Tag ID in view 
        tx = nt_table.getEntry("tx").getDouble(0.0);
        ty = nt_table.getEntry("ty").getDouble(0.0);
        ta = nt_table.getEntry("ta").getDouble(0.0);
    }
}
