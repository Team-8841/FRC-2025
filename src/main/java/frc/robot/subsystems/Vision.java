package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private NetworkTable fwd_table = NetworkTableInstance.getDefault().getTable("limelight-fwd");
    private NetworkTableEntry fwd_tx = fwd_table.getEntry("tx");
    private NetworkTableEntry fwd_ty = fwd_table.getEntry("ty");
    private NetworkTableEntry fwd_ta = fwd_table.getEntry("ta");

    private NetworkTable rear_table = NetworkTableInstance.getDefault().getTable("limelight-rear");
    private NetworkTableEntry rear_tx = rear_table.getEntry("tx");
    private NetworkTableEntry rear_ty = rear_table.getEntry("ty");
    private NetworkTableEntry rear_ta = rear_table.getEntry("ta");

    public Vision() {

        //Shuffleboard.getTab("Driver Data").addCamera("#[Driver View]", "limelight-fwd", "mjpeg:http://10.88.41.13:5800")
        //.withProperties(Map.of("showControls", "false"));

    }
    




    @Override
    public void periodic() {
        
    }
}
