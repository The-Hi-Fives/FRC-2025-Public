package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFXFactory;

public class ElevatorSubsystem extends SubsystemBase {
  
  private TalonFX m_motor =
              TalonFXFactory.createTalon(Constants.Elevator.motorID,
                    Constants.Elevator.motorCANBus, Constants.Elevator.configuration);

  private final DynamicMotionMagicVoltage m_dynamicMotionMagicControl =
      new DynamicMotionMagicVoltage(0, 0, 0, 0);
          
  private double m_targetHeight = Constants.Elevator.minHeight;

  public double getHeight() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public void setHeight(double targetHeight) {
    m_targetHeight = targetHeight;
  }

  @Override
  public void periodic() {
    setDynamicMotionMagicPositionSetpoint(
        Constants.Elevator.motorPositionSlot,
        m_targetHeight,
        Constants.Elevator.maxVelocity,
        Constants.Elevator.maxAcceleration,
        Constants.Elevator.maxJerk,
        0.0);
    SmartDashboard.putNumber("Elevator/Current Height", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Target Height", m_motor.getClosedLoopReference().getValueAsDouble());
  }

  public void setDynamicMotionMagicPositionSetpoint(
      final int slot,
      final double setpoint,
      final double velocity,
      final double acceleration,
      final double jerk,
      final double feedforwardVolts) {
    m_dynamicMotionMagicControl.Slot = slot;
    m_dynamicMotionMagicControl.Position = setpoint;
    m_dynamicMotionMagicControl.FeedForward = feedforwardVolts;
    m_dynamicMotionMagicControl.Velocity = velocity;
    m_dynamicMotionMagicControl.Acceleration = acceleration;
    m_dynamicMotionMagicControl.Jerk = jerk;
    m_motor.setControl(m_dynamicMotionMagicControl);
  }
}