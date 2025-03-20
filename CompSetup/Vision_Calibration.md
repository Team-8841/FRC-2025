# Vision Calibration
1. Connect to Robot WiFi Network (XXXXXX)
2. Open a Web Browser
3. Type http://10.88.44.13:5801 or Click the link
4. Ensure LimeLight LEDS are enabled. **Input** tab -> LEDS -> On
5. Align the Robot to the one side of the reef. 
> Note: Left/Right references the perspective of a robot looking at the reef head on. April Tag ID number does not matter for calibration.
```

   Left  Right
    |     |
    |     | 
    |     |
    |     |
|-----_-----|
|    |_|    | 
|     ^     |
|_____|_____|
      |
   April Tag
```
6. From the Limelight Page go to **Advanced** tab
7. Ensure the view is configured to Target Pose in Robot Space
8. Record the values for TZ, TX, RY
9. Repeat this measurement for the Robot positioned on the other Reef structure.
10. Use these value to update the src/main/java/Constants.java LEFT_CORAL_OFFSETS and RIGHT_CORAL_OFFSETS

## Example Configuration

```java
public static final double[] LEFT_CORAL_OFFSETS = {
    .5,  // Update TZ 
    .14, // Update TX 
    2.0  // Update RY
    };

public static final double[] RIGHT_CORAL_OFFSETS = {
    0.5,  // Update TZ
    -0.2, // Update TX
    0.8   // Update RY
    };
```

# Lime light Calibration
*This should only need to be performed if the Limelight is bumped or removed and replaced.*

Camera Calibration (Only should be required if Lime Light is moved or bumped)
  1) Push robot against a vertical April Tag, and center tag on the front bumper.
  2) Open the Lime Light https://<IP>:5801 web interface.
  3) Go to Advanced and makes sure the 3d visualization is set to targepose in robotspace 
  4) Adjust the offset in Constansts.java -> LimelightConstants 
        The goal is for the visualization to see the April tag as positioned centered and at the bumper relative to the robot in 3d space.
  5) Notes about constants
	All these offsets are intended to provide position of the camera relative to robot center.

	*** All values are in meters ****	
	FWD_OFFSET // Positive, probably doesn't need a ton of adjustment. 
	SIDE_OFFSET // Positive, Distance from vertical centerline
	HEIGHT_OFFSET // Positive, Distance from ground to camera lens
	Roll // Positive, adjust until the visualized bumper is perpendicular to April tag
	Pitch // Negative, this roughly zeros out one of the on screen values... totally blanking on which one sorry
	Yaw //Positive, Should zero out the RY value, I'm 90% sure.