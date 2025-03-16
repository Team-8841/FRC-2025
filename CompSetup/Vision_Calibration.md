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