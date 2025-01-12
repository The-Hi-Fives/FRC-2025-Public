package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public static final class ExampleElevator {
    public static final int motorID = 99;
    public static final String motorCANBus = "rio";
    public static final int motorPositionSlot = 0;
    public static final double maxVelocity = 2.0; // rot/s
    public static final double maxAcceleration = 30.0; // rot/s^2
    public static final double maxJerk = 0.0; // rot/s^3
    public static final double minHeight = 0.0; // rot
    public static final double maxHeight = 16.0; // rot
    public static final double elevatorExtendHeight = 10.0; // rot

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(80)
          .withSupplyCurrentLimit(40)
          .withStatorCurrentLimitEnable(false)
          .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(
        new Slot0Configs()
          .withKV(0)
          .withKA(0)
          .withKP(15)
          .withKI(0)
          .withKD(0)
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKG(0.28));
  }
  public static final class ExampleArm {
    public static final int motorID = 99;
    public static final String motorCANBus = "rio";
    public static final int motorPositionSlot = 0;
    public static final double maxVelocity = 2.0; // rot/s
    public static final double maxAcceleration = 30.0; // rot/s^2
    public static final double maxJerk = 0.0; // rot/s^3
    public static final double minHeight = 0.0; // rot
    public static final double maxHeight = 16.0; // rot
    public static final double armExtendHeight = 10.0; // rot

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(false))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(
          new Slot0Configs()
            .withKV(0)
            .withKA(0)
            .withKP(15)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(0.28));
  }

  public static final class ExampleFlywheel {
    public static final int motorID = 99;
    public static final String motorCANBus = "rio";
    public static final int motorPositionSlot = 0;
    public static final double maxVelocity = 2.0; // rot/s
    public static final double maxAcceleration = 30.0; // rot/s^2
    public static final double maxJerk = 0.0; // rot/s^3
    public static final double minTargetVelocity = 0.0; // rot
    public static final double maxTargetVelocity = 16.0; // rot
    public static final double exampleTargetVelocity = 10.0; // rot

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(80)
          .withSupplyCurrentLimit(40)
          .withStatorCurrentLimitEnable(false)
          .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(
        new Slot0Configs()
          .withKV(0)
          .withKA(0)
          .withKP(15)
          .withKI(0)
          .withKD(0)
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKG(0.28));
  }

  public static final class ExampleRoller {
    public static final int motorID = 99;
    public static final String motorCANBus = "rio";
    public static final double minOutput = 0.0; // proportion 0 to 1
    public static final double minVoltage = 0.0; // volts -12 to 12
    public static final double exampleOutput = 0.65; // proportion 0 to 1
    public static final double exampleVoltage = 10.0; // volts -12 to 12

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
    .withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive));
  }
}
