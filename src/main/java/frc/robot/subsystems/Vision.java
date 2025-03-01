package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private NetworkTable fwd_table = NetworkTableInstance.getDefault().getTable("fwd_vision");
    private NetworkTableEntry fwd_tx = fwd_table.getEntry("tx");
    private NetworkTableEntry fwd_ty = fwd_table.getEntry("ty");
    private NetworkTableEntry fwd_ta = fwd_table.getEntry("ta");

    private NetworkTable rear_table = NetworkTableInstance.getDefault().getTable("rear_vision");
    private NetworkTableEntry rear_tx = rear_table.getEntry("tx");
    private NetworkTableEntry rear_ty = rear_table.getEntry("ty");
    private NetworkTableEntry rear_ta = rear_table.getEntry("ta");

    public Vision() {
        CameraServer.addServer("# fwd_vision", 5800);
    }
    




    @Override
    public void periodic() {
        
    }
}
