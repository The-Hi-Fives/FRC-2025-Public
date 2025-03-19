package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends TimedRobot {
  private static final int CANDLE_ID = 16; // Set the CANdle device ID
  private final CANdle candle = new CANdle(CANDLE_ID);

  // Xbox Controller for controlling LED effects
  private final XboxController controller = new XboxController(0);

  // Default colors
  private static final int[] DISABLED_COLOR = {128, 128, 128}; // Dim white when disabled
  private static final int[] IDLE_COLOR = {0, 0, 255}; // Blue when enabled & idling

  @Override
  public void robotInit() {
    configureCANdle();
  }

  private void configureCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // Set according to your LED strip type (RGB or RGBW)
    config.brightnessScalar = 0.5; // Adjust brightness (0.0 to 1.0)
    candle.configAllSettings(config);
  }

  @Override
  public void teleopInit() {
    setSolidColor(IDLE_COLOR[0], IDLE_COLOR[1], IDLE_COLOR[2]); // Set idle color when enabled
  }

  @Override
  public void teleopPeriodic() {
    // Check for driver input; if no button is pressed, stay in idle color
    if (controller.getAButton()) {
      setSolidColor(255, 0, 0); // Red
    } else if (controller.getBButton()) {
      setSolidColor(0, 255, 0); // Green
    } else if (controller.getXButton()) {
      setSolidColor(0, 0, 255); // Blue
    } else if (controller.getYButton()) {
      animateRainbow();
    } else if (controller.getRightBumper()) {
      animateStrobe();
    } else {
      // If no buttons are pressed, stay at idle color
      setSolidColor(IDLE_COLOR[0], IDLE_COLOR[1], IDLE_COLOR[2]);
    }

    SmartDashboard.putString("LED Status", "Active");
  }

  @Override
  public void disabledInit() {
    setSolidColor(DISABLED_COLOR[0], DISABLED_COLOR[1], DISABLED_COLOR[2]); // Set disabled color
  }

  // Method to set the LED to a solid color
  private void setSolidColor(int r, int g, int b) {
    candle.setLEDs(r, g, b); // Set color to all LEDs
  }

  // Method to animate a rainbow effect
  private void animateRainbow() {
    RainbowAnimation rainbow = new RainbowAnimation(1, 0.5, 64); // Speed, brightness, # LEDs
    candle.animate(rainbow);
  }

  // Method to create a strobe effect
  private void animateStrobe() {
    StrobeAnimation strobe = new StrobeAnimation(255, 255, 255, 0, 0.2, 64); // White strobe
    candle.animate(strobe);
  }
}
