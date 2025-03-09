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

    private NetworkTable fwd_table = NetworkTableInstance.getDefault().getTable("limelight-fwd");
    private NetworkTableEntry fwd_tx = fwd_table.getEntry("tx");
    private NetworkTableEntry fwd_ty = fwd_table.getEntry("ty");
    private NetworkTableEntry fwd_ta = fwd_table.getEntry("ta");
    private NetworkTableEntry tid = fwd_table.getEntry("tid"); // Primary April Tag ID in view
    private double[] botpose_orb = fwd_table.getEntry("bostpose_orb").getDoubleArray(new double[6]);
    private double [] targetpose_robotspace = fwd_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    private double [] targetpose_cameraspace = fwd_table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    private double [] botpose_targetspace = fwd_table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    private NetworkTable rear_table = NetworkTableInstance.getDefault().getTable("limelight-rear");
    private NetworkTableEntry rear_tx = rear_table.getEntry("tx");
    private NetworkTableEntry rear_ty = rear_table.getEntry("ty");
    private NetworkTableEntry rear_ta = rear_table.getEntry("ta");

    public Vision() {

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
    

  



    @Override
    public void periodic() {
        
    }
}
