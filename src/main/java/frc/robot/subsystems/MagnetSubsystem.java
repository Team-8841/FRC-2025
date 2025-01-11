package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagnetSubsystem extends SubsystemBase{

    PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);



    public MagnetSubsystem() {
        setMagnet(false);

    }

    @Override
    public void periodic(){
        updateStatus();
    }


    public void setMagnet(boolean state){
        PDH.setSwitchableChannel(state);
    }


    private void updateStatus() {
        SmartDashboard.putBoolean("Magnet", PDH.getSwitchableChannel());
    }
    
}
