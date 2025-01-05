package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This class allows for variables to be updated from the SmartDashboard
// Values placed here are adjusted to fine tune robot functionality and pushed via SmartDashboard
// Ex. PID values

// *** All values below may be modified during operation ***

public class Tuning {
  //Values 
  boolean telemStatus; // Used to show SmartDashboard connection status

  // Called during robot init, values are inital values
  public Tuning () {
    telemStatus = true;
    
  }


  // Initalize values 
  public void initValues() {
    SmartDashboard.putBoolean("telemStatus", telemStatus);
  }

  // Update 
  public void updateValues() {
    
  }
}


