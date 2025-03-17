package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCam extends SubsystemBase {
    
    private ShuffleboardTab m_camTab;
    private GenericEntry m_exposure, m_whiteBalance;
    private UsbCamera m_cam;


    public DriverCam() {

        m_cam = CameraServer.startAutomaticCapture("#DriverCam", 0);
        m_cam.setFPS(15);
        m_cam.setResolution(320, 240);
        m_cam.setExposureManual(40); // Change this value to adjust exposure
        m_cam.setExposureHoldCurrent();
        m_cam.setWhiteBalanceManual(0); // Change this value to adjust white balance
        m_cam.setWhiteBalanceHoldCurrent();

    }

    @Override
    public void periodic() {
        
    }
 
}
