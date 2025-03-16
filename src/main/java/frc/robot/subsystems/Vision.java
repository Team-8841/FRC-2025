package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase{

    private String ll_name;
    private NetworkTable nt_table;
    private boolean targetRightCoral;

    private double TX, TY, ROT;
    
    private long tid;
    private double[] targetpose_robotspace = new double[6];
    private double[] botpose_targetspace = new double[6];
    private double[] positions = new double[6];

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

    public boolean isTargetRightCoral()
    {
        return targetRightCoral;
    }    

    public String getLLName()
    {
        return ll_name;
    }

    @Override
    public void periodic() {
        botpose_targetspace = nt_table.getEntry("botpose_targetspace").getDoubleArray(new double [6]);
        positions = LimelightHelpers.getBotPose_TargetSpace(ll_name);
        //SmartDashboard.putNumber("$[LL]RAW_TX_CURRENT", positions[2]);
        //SmartDashboard.putNumber("$[LL]RAW_TY_CURRENT", positions[0]);
        //SmartDashboard.putNumber("$[LL]RAW_ROT_CURRENT", positions[4]);
        double[][] sampled_positions = new double[LimelightConstants.LL_SAMPLING][6];
        for (int i = 0; i < LimelightConstants.LL_SAMPLING; i++){
            sampled_positions[i] = LimelightHelpers.getBotPose_TargetSpace(ll_name);
        }
        for (int r = 0; r < 6; r++){   
            double avg = 0;
            for(int i = 0; i < LimelightConstants.LL_SAMPLING; i++)
            {
                avg = avg + sampled_positions[i][r];
            }
            positions[r] = avg/LimelightConstants.LL_SAMPLING;
        }

        //SmartDashboard.putNumber("$[LL]AVG_TX_CURRENT", positions[2]);
        //SmartDashboard.putNumber("$[LL]AVG_TY_CURRENT", positions[0]);
        //SmartDashboard.putNumber("$[LL]AVG_ROT_CURRENT", positions[4]);

        tid = nt_table.getEntry("tid").getInteger(0); // Primary April Tag ID in view 
    }
}
