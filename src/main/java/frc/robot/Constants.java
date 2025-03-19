// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file)..
 */
public final class Constants {
  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 5;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -1;
    public static final double CLIMBER_SPEED_UP = 1;
  }

  public static final class ElevatorConstants {

    public static final int motorID = 13; // ELevator 1
    public static final int motorID2 = 14; // Elevator 2
    public static final String motorCANBus = "rio";

    public static final double elevatorGearRatio = 1 / 9.241; // Sensor to Mechanism Ratio
    public static final double elevatorPinionRadius = Units.inchesToMeters(0.0178); // Meters
    public static final double motorPositionSlot = 0;

    public static final double maxElevatorHeight = 2; // Meters
    public static final double elevatorExtendHeight = 1;

    // all edits were courtusy of SuperNURDS 3255

    public static final TalonFXConfiguration configuration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(0.22)
                    .withSupplyCurrentLimit(0.22)
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKV(0.001) // originally was 0
                    .withKA(0)
                    .withKP(
                        0.03) // change to 0.5 after testing whats entered, if not problems concure
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.4) // new line
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKG(0.3))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(elevatorGearRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(400) // originally was .88
                    .withMotionMagicAcceleration(500) // originally was .88
                    .withMotionMagicExpo_kV(0.12) // new line
                    .withMotionMagicJerk(50))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(elevatorMetersToRotations(maxElevatorHeight)));

    public static final PositionVoltage elevatorPositionControl = new PositionVoltage(0);

    public static final Follower followerControl = new Follower(motorID, false);

    public static final double heightErrorTolerance = 0.0025; // Meters

    public static final double kElevatorFastUpdateFrequency = 50; // Hertz
    public static final double kElevatorMidUpdateFrequency = 40; // Hertz

    public static double elevatorMetersToRotations(double meters) {

      return meters / (2 * Math.PI * elevatorPinionRadius);
    }

    public static double elevatorRotationsToMeters(double rotations) {

      return rotations * (2 * Math.PI * elevatorPinionRadius);
    }
  }

  public static final class IntakeConstants {

    public static final int intakeTalonID = 17;
    public static final String intakeTalonCANBus = "rio";

    public static final int intakeSensorID = 1;
    public static final RangingMode intakeSensorRange = RangingMode.Short;
    public static final double intakeSampleTime = 0;

    public static final double isCoralPresentTOF = 76.2; // Milimeters

    public static final TalonFXConfiguration kIntakeConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(200)
                    .withSupplyCurrentLimit(200)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive));

    public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);
    public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0);
  }

  public static final class WristConstants {
    public static final int armLeaderID = 18;
    public static final String armTalonCANBus = "rio";
    public static final double armGearRatio = (88.392857142856982); // Sensor to Mechanism Ratio

    public static final double armMinClamp = -500;
    public static final double armMaxClamp = 500;

    public static final Rotation2d armMinAngle = Rotation2d.fromDegrees(armMinClamp);
    public static final Rotation2d armMaxAngle = Rotation2d.fromDegrees(armMaxClamp);

    public static final TalonFXConfiguration kArmConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100)
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.1)
                    .withMotionMagicAcceleration(0.1)
                    .withMotionMagicJerk(0))
            .withSlot0(
                new Slot0Configs()
                    .withKV(0) // 0
                    .withKA(0) // 0
                    .withKP(70) // 70
                    .withKI(0.05) // 0.05
                    .withKD(0.1) // 0.1
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKG(0)
                    .withKS(0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(armGearRatio))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withReverseSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(armMaxAngle.getRotations())
                    .withReverseSoftLimitThreshold(armMinAngle.getRotations()));

    public static final MotionMagicExpoVoltage armPositionControl = new MotionMagicExpoVoltage(0);
    public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(1); // Degrees
  }

  public static class Setpoints {
    public static final double ElevatorStowHeight = 0;
  }

  public static final double kConfigTimeoutSeconds = 0.1;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
