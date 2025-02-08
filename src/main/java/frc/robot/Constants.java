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
import edu.wpi.first.units.measure.Angle;

public class Constants {
  public static final class ElevatorConstants {

    public static final int motorID = 13; //ELevator 1
    public static final int motorID2 = 14; //Elevator 2
    public static final String motorCANBus = "rio";

    public static final double elevatorGearRatio = 1/9.241; // Sensor to Mechanism Ratio
    public static final double elevatorPinionRadius = Units.inchesToMeters(0.0178); // Meters
    public static final double motorPositionSlot = 0;

    public static final double maxElevatorHeight = 1; // Meters
    public static final double elevatorExtendHeight = 1;

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(0.22)
        .withSupplyCurrentLimit(0.22)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(new Slot0Configs()
        .withKV(0.001) //originally was 0
        .withKA(0)
        .withKP(0.013) //change to 0.5 after testing whats entered, if not problems concure
        .withKI(0)
        .withKD(0)
        .withKS(0.4) //new line
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.3))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(elevatorGearRatio))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(400) //originally was .88
        .withMotionMagicAcceleration(1100) //originally was .88
        .withMotionMagicExpo_kV(0.12) //new line
        .withMotionMagicJerk(0))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
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

    //ALL COMMENTED OUT CODE IT TEMPORARY.

    //public static final int intakeSensorID = 1;
    //public static final RangingMode intakeSensorRange = RangingMode.Short;
    //public static final double intakeSampleTime = 0;

    //public static final double isNotePresentTOF = 350; // Milimeters

    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive));


    public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);
    public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0);
  }


  public static final class WristConstants {
        public static final int armLeaderID = 14; //right
        public static final String armTalonCANBus = "rio";
        public static final double armGearRatio = 88.3929; // Sensor to Mechanism Ratio

        public static final double ArmIntakeAngle = 10;

        public static final double armMinClamp = -10;
        public static final double armMaxClamp = 360;

        public static final Rotation2d armMinAngle = Rotation2d.fromDegrees(armMinClamp);
        public static final Rotation2d armMaxAngle = Rotation2d.fromDegrees(armMaxClamp);

        public static final TalonFXConfiguration kArmConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0.22)
        .withMotionMagicAcceleration(0.22)
        .withMotionMagicJerk(0))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(1) 
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0)
        .withKS(0))
      .withFeedback(new FeedbackConfigs()
      .withSensorToMechanismRatio(armGearRatio))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(armMaxAngle.getRotations())
        .withReverseSoftLimitThreshold(armMinAngle.getRotations()));

        public static final MotionMagicExpoVoltage armPositionControl = new MotionMagicExpoVoltage(0);
        public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(2.5); // Degrees
  }
  public static final class ExampleFlywheel {
    public static final int motorID = 99;
    public static final String motorCANBus = "CANivore";
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


  public static class Setpoints {

    public static final double ElevatorStowHeight = 0;

  }
    
    public static final double kConfigTimeoutSeconds = 0.1;


}
