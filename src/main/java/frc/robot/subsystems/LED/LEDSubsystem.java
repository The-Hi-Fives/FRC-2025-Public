// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  // Control everything with a CANdle
  private static final CANdle m_candle = new CANdle(16, "rio");

  /*
   * Robot LED States
   */
  private static enum LEDState {
    START,
    DISABLED,
    DISABLED_LOW_BATTERY,
    DISABLED_TARGET,
    AUTONOMOUS,
    ENABLED,
    INTAKING,
    FEEDING_SHOOTER,
    FEEDING_AMP,
    CLIMBING,
    HAVENOTE_INTAKE,
    HAVENOTE_SCORING,
    AMPING,
    SHOOTING_SEQUENCE,
    READY_TO_SHOOT
  }

  LEDState m_currentState = LEDState.START;

  /*
   * Colors
   */
  class Color {
    int r, g, b;

    private Color(int red, int green, int blue) {
      this.r = red;
      this.g = green;
      this.b = blue;
    }
  }

  Color black = new Color(0, 0, 0); // This will Turn off the CANdle
  Color white = new Color(255, 255, 255);
  Color red = new Color(255, 0, 0);
  Color green = new Color(0, 255, 0);
  Color blue = new Color(0, 0, 255);
  Color yellow = new Color(255, 255, 0);
  Color cyan = new Color(0, 255, 240);
  Color magenta = new Color(255, 0, 255);
  Color brown = new Color(166, 41, 41);
  Color pink = new Color(255, 60, 150);
  Color purple = new Color(170, 0, 255);

  /*
   * LED Segments
   */
  class LEDSegment {

    int startIndex;
    int segmentSize;
    int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      m_candle.clearAnimation(animationSlot);
      m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
      m_candle.modulateVBatOutput(0.95);
    }

    private void setAnimation(Animation animation) {
      m_candle.clearAnimation(animationSlot);
      m_candle.animate(animation, animationSlot);
      m_candle.modulateVBatOutput(0.95);
    }

    public void setOff() {
      m_candle.clearAnimation(animationSlot);
      m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
      m_candle.modulateVBatOutput(0.0);
    }
  }

  /*
   * Constructor
   * Creates a new LEDSubsystem
   */
  public LEDSubsystem() {

    m_candle.configFactoryDefault();

    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    m_candle.getAllConfigs(candleConfiguration);
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.GRB;
    candleConfiguration.brightnessScalar = 0.9;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(candleConfiguration, 100);

    m_candle.getAllConfigs(candleConfiguration);

    m_candle.configLEDType(LEDStripType.GRB, 300);

    m_candle.getAllConfigs(candleConfiguration);
  }

  @Override
  public void periodic() {
    LEDState newState = LEDState.DISABLED;

    if (DriverStation.isDisabled()) {
      if (RobotController.getBatteryVoltage() < 11.8) {
        newState = LEDState.DISABLED_LOW_BATTERY;
      } else {
        newState = LEDState.DISABLED;
      }

    } else if (DriverStation.isAutonomousEnabled()) {
      newState = LEDState.AUTONOMOUS;

    } else {
      // Just Enabled
      newState = LEDState.ENABLED;
    }
  }

  private void LEDStateMachine(LEDState newState) {

    switch (newState) {
      case DISABLED:
        m_Matrix.setColor(black);
        m_VerticalRight.setColor(black);
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            m_ChassisRight.setAnimation(a_LeftBlueFlow);
            m_VerticalRight.setAnimation(a_RightBlueFlow);
          } else {
            m_ChassisRight.setAnimation(a_LeftRedFlow);
            m_VerticalRight.setAnimation(a_RightRedFlow);
          }
        }
        break;

      case DISABLED_LOW_BATTERY:
        m_Matrix.setOff();
        m_VerticalLeft.setAnimation(a_BrownLeftStrobe);
        m_VerticalRight.setAnimation(a_BrownRightStrobe);
        m_ChassisLeft.setAnimation(a_BrownChassisLeftStrobe);
        m_ChassisRight.setAnimation(a_BrownChassisRightStrobe);
        m_HorizontalStrip.setAnimation(a_BrownHorizontalStrobe);
        break;

      case AUTONOMOUS:
        m_Matrix.setOff();
        m_VerticalLeft.setAnimation(a_LeftFlame);
        m_VerticalRight.setAnimation(a_RightFlame);
        m_ChassisLeft.setColor(white);
        m_ChassisLeft.setColor(white);
        break;

      case ENABLED:
        m_Matrix.setOff();
        m_ChassisLeft.setColor(white);
        m_ChassisRight.setColor(white);
        m_VerticalLeft.setColor(white);
        m_VerticalRight.setColor(white);
        m_HorizontalStrip.setColor(white);
        break;

      default:
        break;
    }
  }

  public void setBrightness(double percent) {
    /* Here we will set the brightness of the LEDs */
    m_candle.configBrightnessScalar(percent, 100);
  }

  public void fullClear() {
    clearAnimations();
    disableLEDs();
    m_candle.modulateVBatOutput(0.0);
  }

  public void clearAnimations() {
    m_candle.clearAnimation(m_Matrix.animationSlot);
    m_candle.clearAnimation(m_VerticalRight.animationSlot);
    m_candle.clearAnimation(m_ChassisRight.animationSlot);
  }

  public void disableLEDs() {
    m_Matrix.setOff();
    m_VerticalRight.setOff();
    m_ChassisRight.setOff();
  }

  LEDSegment m_Matrix = new LEDSegment(0, 8, 0);
  LEDSegment m_ChassisRight = new LEDSegment(9, 34, 1);
  LEDSegment m_VerticalRight = new LEDSegment(43, 23, 2);
  LEDSegment m_HorizontalStrip = new LEDSegment(66, 24, 3);
  LEDSegment m_VerticalLeft = new LEDSegment(90, 23, 4);
  LEDSegment m_ChassisLeft = new LEDSegment(113, 34, 5);

  Animation a_WhiteLeftStrobe =
      new StrobeAnimation(
          white.r,
          white.g,
          white.b,
          0,
          0.5,
          m_VerticalLeft.segmentSize,
          m_VerticalLeft.startIndex); // Flash
  Animation a_WhiteRightStrobe =
      new StrobeAnimation(
          white.r,
          white.g,
          white.b,
          0,
          0.5,
          m_VerticalRight.segmentSize,
          m_VerticalRight.startIndex);

  Animation a_GreenLeftStrobe =
      new StrobeAnimation(
          green.r,
          green.g,
          green.b,
          0,
          0.7,
          m_VerticalLeft.segmentSize,
          m_VerticalLeft.startIndex); // Flash
  Animation a_GreenRightStrobe =
      new StrobeAnimation(
          green.r,
          green.g,
          green.b,
          0,
          0.7,
          m_VerticalRight.segmentSize,
          m_VerticalRight.startIndex);

  Animation a_BrownLeftStrobe =
      new StrobeAnimation(
          brown.r,
          brown.g,
          brown.b,
          0,
          0.2,
          m_VerticalLeft.segmentSize,
          m_VerticalLeft.startIndex); // Flash
  Animation a_BrownRightStrobe =
      new StrobeAnimation(
          brown.r,
          brown.g,
          brown.b,
          0,
          0.2,
          m_VerticalRight.segmentSize,
          m_VerticalRight.startIndex);
  Animation a_BrownChassisLeftStrobe =
      new StrobeAnimation(
          brown.r, brown.g, brown.b, 0, 0.2, m_ChassisRight.segmentSize, m_ChassisRight.startIndex);
  Animation a_BrownChassisRightStrobe =
      new StrobeAnimation(
          brown.r, brown.g, brown.b, 0, 0.2, m_ChassisLeft.segmentSize, m_ChassisLeft.startIndex);
  Animation a_BrownHorizontalStrobe =
      new StrobeAnimation(
          brown.r,
          brown.g,
          brown.b,
          0,
          0.2,
          m_HorizontalStrip.segmentSize,
          m_HorizontalStrip.startIndex);

  Animation a_CyanLeftPingPong =
      new LarsonAnimation(
          cyan.r,
          cyan.g,
          cyan.b,
          0,
          0.8,
          m_VerticalLeft.segmentSize,
          BounceMode.Back,
          6,
          m_VerticalLeft.startIndex);
  Animation a_CyanRightPingPong =
      new LarsonAnimation(
          cyan.r,
          cyan.g,
          cyan.b,
          0,
          0.8,
          m_VerticalRight.segmentSize,
          BounceMode.Back,
          6,
          m_VerticalRight.startIndex);

  Animation a_PinkLeftPingPong =
      new LarsonAnimation(
          pink.r,
          pink.g,
          pink.b,
          0,
          0.8,
          m_VerticalLeft.segmentSize,
          BounceMode.Back,
          6,
          m_VerticalLeft.startIndex);
  Animation a_PinkRightPingPong =
      new LarsonAnimation(
          pink.r,
          pink.g,
          pink.b,
          0,
          0.8,
          m_VerticalRight.segmentSize,
          BounceMode.Back,
          6,
          m_VerticalRight.startIndex);

  Animation a_LeftRainbow =
      new RainbowAnimation(0.7, 0.5, m_VerticalLeft.segmentSize, true, m_VerticalLeft.startIndex);
  Animation a_RightRainbow =
      new RainbowAnimation(
          0.7, 0.5, m_VerticalRight.segmentSize, false, m_VerticalRight.startIndex);

  Animation a_LeftFlame =
      new FireAnimation(
          0.9, 0.75, m_VerticalLeft.segmentSize, 1.0, 0.3, true, m_VerticalLeft.startIndex);
  Animation a_RightFlame =
      new FireAnimation(
          0.9, 0.75, m_VerticalRight.segmentSize, 1.0, 0.3, false, m_VerticalRight.startIndex);

  Animation a_LeftRedFlow =
      new ColorFlowAnimation(
          red.r,
          red.g,
          red.b,
          0,
          0.7,
          m_VerticalLeft.segmentSize,
          Direction.Backward,
          m_VerticalLeft.startIndex);
  Animation a_RightRedFlow =
      new ColorFlowAnimation(
          red.r,
          red.g,
          red.b,
          0,
          0.7,
          m_VerticalRight.segmentSize,
          Direction.Forward,
          m_VerticalRight.startIndex);

  Animation a_LeftBlueFlow =
      new ColorFlowAnimation(
          blue.r,
          blue.g,
          blue.b,
          0,
          0.7,
          m_VerticalLeft.segmentSize,
          Direction.Backward,
          m_VerticalLeft.startIndex);
  Animation a_RightBlueFlow =
      new ColorFlowAnimation(
          blue.r,
          blue.g,
          blue.b,
          0,
          0.7,
          m_VerticalRight.segmentSize,
          Direction.Forward,
          m_VerticalRight.startIndex);

  Animation a_LeftCyanFlow =
      new ColorFlowAnimation(
          cyan.r,
          cyan.g,
          cyan.b,
          0,
          0.95,
          m_VerticalLeft.segmentSize,
          Direction.Backward,
          m_VerticalLeft.startIndex);
  Animation a_RightCyanFlow =
      new ColorFlowAnimation(
          cyan.r,
          cyan.g,
          cyan.b,
          0,
          0.95,
          m_VerticalRight.segmentSize,
          Direction.Forward,
          m_VerticalRight.startIndex);

  Animation a_LeftPurpleFlow =
      new ColorFlowAnimation(
          purple.r,
          purple.g,
          purple.b,
          0,
          0.95,
          m_VerticalLeft.segmentSize,
          Direction.Backward,
          m_VerticalLeft.startIndex);
  Animation a_RightPurpleFlow =
      new ColorFlowAnimation(
          purple.r,
          purple.g,
          purple.b,
          0,
          0.95,
          m_VerticalRight.segmentSize,
          Direction.Forward,
          m_VerticalRight.startIndex);

  Animation a_LeftGreenTwinkle =
      new ColorFlowAnimation(
          green.r,
          green.g,
          green.b,
          0,
          0.2,
          m_VerticalLeft.segmentSize,
          Direction.Backward,
          m_VerticalLeft.startIndex);
  Animation a_RightGreenTwinkle =
      new ColorFlowAnimation(
          green.r,
          green.g,
          green.b,
          0,
          0.2,
          m_VerticalRight.segmentSize,
          Direction.Forward,
          m_VerticalRight.startIndex);
}
