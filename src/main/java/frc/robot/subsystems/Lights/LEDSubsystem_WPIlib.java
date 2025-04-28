package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

//LED Imports
//import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.AddressableLEDBufferView;

public class LEDSubsystem_WPIlib extends SubsystemBase {
  // edu.wpi.first.wpilibj.AddressableLED
  // A class for driving addressable LEDs, such as WS2812Bs and NeoPixels.
  // By default, the timing supports WS2812B LEDs, but is configurable using
  // setBitTiming()
  // Only 1 LED driver is currently supported by the roboRIO.
  // However, multiple LED strips can be connected in series and controlled from
  // the single driver.

  // Setup
  private static final int kPort = LightsConstants.port; // PWM Port
  private static final int kLength = LightsConstants.length; // LED strip length [# of LEDs]
  private static final Distance kLedSpacing = LightsConstants.spacing; // LED strip LEDs density - [... LEDs per meter]
  private static final int brightness = LightsConstants.brightness;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledbuffer;
  // private final AddressableLEDBufferView m_left; //Left side of the LED strip
  // private final AddressableLEDBufferView m_right; //Right side of the LED strip
  private final Timer timer = new Timer(); // WPILib Timer
  private final Random random = new Random();
  private boolean running_AnimatedPattern = false;
  private boolean running_TwinklePattern = false;
  private double twinklePeriod = 0;
  private Color twinkleBaseColor = null;
  private List<Integer> twinkleIndexes = new ArrayList<>();
  private LEDPattern animatedPattern;

  public LEDSubsystem_WPIlib() {
    m_led = new AddressableLED(kPort);
    m_ledbuffer = new AddressableLEDBuffer(kLength);
    // m_left = m_ledbuffer.createView(0, kLength/2 - 1); //Left side of the LED
    // strip
    // m_right = m_ledbuffer.createView(kLength/2, kLength - 1).reversed(); //Right
    // side of the LED strip
    m_led.setLength(m_ledbuffer.getLength());
    m_led.setData(m_ledbuffer);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    running_AnimatedPattern = false;
    running_TwinklePattern = false;
    twinkleBaseColor = null;
    randomizeList();
    twinklePeriod = 0;
    animatedPattern = null;
    // setDefaultCommand(LED_Reset().withName("LED_Reset"));
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack),
    // false).withName("Off"));

    LED_Twinkle(LightsConstants.GRBColors.get("black"), LightsConstants.GRBColors.get("yellow"), 2);
  }

  /**
   * Disable LED strip - Terminates all patterns and stops the LED strip.
   */
  public void LED_Disable() {
    runPattern(LEDPattern.solid(LightsConstants.GRBColors.get("black")), false);
    m_led.stop();
  }

  /**
   * Resetting LED strip - LES set to solid black.
   */
  public void LED_Reset() {
    runPattern(LEDPattern.solid(LightsConstants.GRBColors.get("black")), false);
  }

  /**
   * Solid color pattern.
   * 
   * @param color the color of the pattern
   */
  public void LED_SolidColor(Color color) {
    runPattern(LEDPattern.solid(color), false);
  }

  /**
   * Scrolling pattern at absolute speed.
   * 
   * @param pattern the LED pattern to run
   * @param speed   the speed of the pattern scrolling in [m/s]
   */
  public void LED_ScrollPatternAbsolute(LEDPattern pattern, double speed) {
    LEDPattern m_scrollingPattern = pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), kLedSpacing);
    runPattern(m_scrollingPattern, true);
    //System.out.println("Scroll function executed!!!!!!!!!!!!");
  }

  /**
   * Blinking pattern.
   * 
   * @param pattern the LED pattern to run
   * @param onTime  the time the LED stays on in [s]
   * @param offTime the time the LED stays off in [s]
   */
  public void LED_Blinking(LEDPattern pattern, double onTime, double offTime) {
    LEDPattern m_blinkingPattern = pattern.blink(Seconds.of(onTime), Seconds.of(offTime));
    runPattern(m_blinkingPattern, true);
  }

  /**
   * Breathing pattern.
   * 
   * @param pattern the LED pattern to run
   * @param period  the time of one full cycle in [s]
   */
  public void LED_Breathing(LEDPattern pattern, double period) {
    LEDPattern m_breathingPattern = pattern.breathe(Seconds.of(period));
    runPattern(m_breathingPattern, true);
  }

  /**
   * Twinkle pattern.
   * 
   * @param baseColor the color of static LED's
   * @param twinkleColor the color of twinkling LED's
   * @param period the time of one full cycle in [s]
   */
  public void LED_Twinkle(Color baseColor, Color twinkleColor, double period) {
    LEDPattern m_breathingPattern = LEDPattern.solid(twinkleColor).breathe(Seconds.of(period));
    runPattern(m_breathingPattern, true);
    timer.reset(); // Ensure clean timer state
    timer.start();
    twinklePeriod = period;
    twinkleBaseColor = baseColor;
    running_TwinklePattern = true;
  }

  /**
   * Stops the twinkle effect by resetting all twinkle-related state.
  */
  public void stopTwinkle() {
    running_TwinklePattern = false;
    twinkleBaseColor = null;
    twinklePeriod = 0;
    twinkleIndexes.clear();
    timer.stop();
    timer.reset();
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    // display
    if (running_AnimatedPattern) {
      animatedPattern.applyTo(m_ledbuffer);
      if (running_TwinklePattern){
        for (int index=0;index<kLength;index++){
          if (!twinkleIndexes.contains(index)){
            m_ledbuffer.setLED(index, twinkleBaseColor);
          }
        }
        if (timer.hasElapsed(twinklePeriod/2)) {
          randomizeList();
          // Reset the timer to start counting again
          timer.reset();
        }
      }
      m_led.setData(m_ledbuffer);
    }
  }


  private void randomizeList() {
    twinkleIndexes.clear();
    int maxTwinkleLEDs = Math.min(3, kLength);
    while (twinkleIndexes.size() < maxTwinkleLEDs) {
        int newNumber = random.nextInt(kLength);
        if (!twinkleIndexes.contains(newNumber)) {
            twinkleIndexes.add(newNumber);
        }
    }
  }
  /**
   * A function that runs a pattern on the entire LED strip.
   * It also controls whether the pattern is animated or not
   * using the running_AnimatedPattern flag and animatedPattern.
   * 
   * 
   * @param pattern  the LED pattern to run
   * @param animated whether the pattern is animated
   * @implNote Recomended pattern input (rainbow): private final LEDPattern
   *           m_rainbow = LEDPattern.rainbow(255, 128);
   */
  public void runPattern(LEDPattern pattern, boolean animated) {
    stopTwinkle();
    if (animated) {
      animatedPattern = pattern.atBrightness(Percent.of(brightness));
      running_AnimatedPattern = true;
    } else {
      animatedPattern = null;
      running_AnimatedPattern = false;
      pattern.atBrightness(Percent.of(brightness)).applyTo(m_ledbuffer);
      m_led.setData(m_ledbuffer);
      // System.out.println("And executed proprely!!!!! Animated:" +
      // running_AnimatedPattern);
    }
    // return run(() -> pattern.applyTo(m_ledbuffer));
  }
}

/**
 * 
 * AddressableLED m_led = new AddressableLED(0);
 * //----------------------------------------------------------------------------------------------------------------------
 * AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
 * m_led.setLength(m_ledBuffer.getLength());
 * 
 * // Set the data
 * m_led.setData(m_ledBuffer);
 * m_led.start();
 * 
 * // Create the view for the section of the strip on the left side of the
 * robot.
 * // This section spans LEDs from index 0 through index 59, inclusive.
 * AddressableLEDBufferView m_left = m_ledBuffer.createView(0, 29);
 * 
 * // The section of the strip on the right side of the robot.
 * // This section spans LEDs from index 60 through index 119, inclusive.
 * // This view is reversed to cancel out the serpentine arrangement of the
 * // physical LED strip on the robot.
 * AddressableLEDBufferView m_right = m_ledBuffer.createView(30, 59).reversed();
 * 
 * //Patterns
 * 
 * //LED pattern that sets the entire strip to solid red
 * LEDPattern red = LEDPattern.solid(Color.kRed);
 * red.applyTo(m_ledBuffer);
 * m_led.setData(m_ledBuffer);
 * 
 * //Animated Rainbow
 * // all hues at maximum saturation and half brightness
 * private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
 * // Our LED strip has a density of 120 LEDs per meter
 * private static final Distance kLedSpacing = Meters.of(1 / 120.0);
 * // Pattern that scrolls the rainbow pattern across the LED strip, moving at a
 * speed
 * // of 1 meter per second.
 * private final LEDPattern m_scrollingRainbow =
 * m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
 * //In periodic
 * m_scrollingRainbow.applyTo(m_ledBuffer);
 * m_led.setData(m_ledBuffer);
 * 
 * //
 */