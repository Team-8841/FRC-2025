package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase{

    private String ll_name, ll_alt_name;
    private NetworkTable nt_table;
    private NetworkTable rightTables, leftTables;
    private boolean targetRightCoral;

    private double TX, TY, ROT;
    
    private long tid;
    private double[] targetpose_robotspace = new double[6];
    private double[] botpose_targetspace = new double[6];
    private double[] positions = new double[6];

    private boolean isRightPrimary, oldState;

    public MiniPID xController, yController, rotController;

    public Vision(String ll_name, String ll_alt_name) {
        this.ll_name = ll_name;
        this.ll_alt_name = ll_alt_name;
        nt_table = NetworkTableInstance.getDefault().getTable(this.ll_name);
        this.rightTables = NetworkTableInstance.getDefault().getTable(this.ll_name);
        this.leftTables = NetworkTableInstance.getDefault().getTable(this.ll_alt_name);
        targetRightCoral = false; //Default target left coral
        // Other reference frame options 
        //private double [] targetpose_cameraspace = fwd_table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        //private double [] botpose_targetspace = fwd_table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace(ll_name, 
            LimelightConstants.FWD_OFFSET,    // Forward offset (meters)
            LimelightConstants.SIDE_OFFSET,    // Side offset (meters)
            LimelightConstants.HEIGHT_OFFSET,    // Height offset (meters)
            LimelightConstants.ROLL,    // Roll (degrees)
            LimelightConstants.PITCH,   // Pitch (degrees)
            LimelightConstants.YAW     // Yaw (degrees)
            );

            LimelightHelpers.setCameraPose_RobotSpace(ll_alt_name, 
            LimelightConstants.FWD_OFFSET_ALT,    // Forward offset (meters)
            LimelightConstants.SIDE_OFFSET_ALT,    // Side offset (meters)
            LimelightConstants.HEIGHT_OFFSET_ALT,    // Height offset (meters)
            LimelightConstants.ROLL_ALT,    // Roll (degrees)
            LimelightConstants.PITCH_ALT,   // Pitch (degrees)
            LimelightConstants.YAW_ALT     // Yaw (degrees)
            );
        //Shuffleboard.getTab("Driver Data").addCamera("#[Driver View]", "limelight-fwd", "mjpeg:http://10.88.41.13:5800")
        //.withProperties(Map.of("showControls", "false"));

        xController = new MiniPID(LimelightConstants.REEF_X_KP, LimelightConstants.REEF_X_KI, LimelightConstants.REEF_X_KI);  // Vertical movement  
        yController = new MiniPID(LimelightConstants.REEF_Y_KP, LimelightConstants.REEF_Y_KI, LimelightConstants.REEF_Y_KI); 
        rotController = new MiniPID(LimelightConstants.REEF_ROT_KP, LimelightConstants.REEF_ROT_KI, LimelightConstants.REEF_ROT_KI);  

        isRightPrimary = true;
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

    public String getPrimaryCam() //Returns primary LL name
    {
        if (isRightPrimary)
        {
            return this.ll_name;
        }
        else
        {
            return this.ll_alt_name;
        }
    }

    public String getCamName(boolean right){
        if(right) {
            return this.ll_name;
        } else {
            return this.ll_alt_name;
        }
    }

    public void setRightLLAsPrimary(boolean state)
    {
        isRightPrimary = state;

        if(LimelightConstants.DEBUG_ENABLED){
            if(state != oldState) {
                oldState = state;
                if(state) {
                    System.out.println("Handed off to Right cam");
                } else {
                    System.out.println("Handed off to Left cam");
                }
            }
        }
    }

    public double getTargetTagID(boolean right) {
        double rightTarget = rightTables.getEntry("tid").getDouble(0);
        double leftTarget = leftTables.getEntry("tid").getDouble(0);
        if (right) {
            return rightTarget;
        } else {
            return leftTarget;
        }
    }

    public void enabledLEDS(boolean state) {
        //These LEDS ARE PAIN ALL I SEE IS DOTS

        if(state) {
            rightTables.getEntry("ledMode").setNumber(3);
            leftTables.getEntry("ledMode").setNumber(3);
        }else {
            rightTables.getEntry("ledMode").setNumber(1);
            leftTables.getEntry("ledMode").setNumber(1);
        }
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
        //System.out.println(sampled_positions[0].length);
        if (sampled_positions[0].length > 0)
        {
        for (int r = 0; r < 6; r++){   
            double avg = 0;
            for(int i = 0; i < LimelightConstants.LL_SAMPLING; i++)
            {
                avg = avg + sampled_positions[i][r];
            }
            positions[r] = avg/LimelightConstants.LL_SAMPLING;
        }
    }

        //SmartDashboard.putNumber("$[LL]AVG_TX_CURRENT", positions[2]);
        //SmartDashboard.putNumber("$[LL]AVG_TY_CURRENT", positions[0]);
        //SmartDashboard.putNumber("$[LL]AVG_ROT_CURRENT", positions[4]);

        if(LimelightConstants.DEBUG_ENABLED) {
            SmartDashboard.putBoolean("$[LEFT LL]: Has Target", LimelightHelpers.getTV(ll_alt_name));
            SmartDashboard.putBoolean("$[Right LL]: Has Target", LimelightHelpers.getTV(ll_name));
            SmartDashboard.putBoolean("$[LL]: Right Primary", isRightPrimary);

            SmartDashboard.putNumber("$[Left LL]: TX", LimelightHelpers.getTX(ll_alt_name));
            SmartDashboard.putNumber("$[Left LL]: TY", LimelightHelpers.getTY(ll_alt_name));
            SmartDashboard.putNumber("$[Left LL]: Target ID", getTargetTagID(false));

            SmartDashboard.putNumber("$[Right LL]: TX", LimelightHelpers.getTX(ll_name));
            SmartDashboard.putNumber("$[Right LL]: TY", LimelightHelpers.getTY(ll_name));
            SmartDashboard.putNumber("$[Right LL]: Target ID", getTargetTagID(true));
        }

   

        tid = nt_table.getEntry("tid").getInteger(0); // Primary April Tag ID in view 
    }
}
