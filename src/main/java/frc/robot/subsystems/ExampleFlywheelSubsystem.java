package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFXFactory;

public class ExampleFlywheelSubsystem extends SubsystemBase {
 /* This is an example Subsystem for a position-controlled single-motor mechanism with an
  * integrated relative encoder. Examples include Shooter Wheels and Flywheels.
  *
  * This example uses TALON FX Motion-Magic Voltage Velocity-Control with units in motor
  * ROTATIONS per SECOND. Example constants for a Flywheel are in Constants.java.
  *
  * https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
  */
  
  //Create motor using TalonFXFactory
  private TalonFX m_motor = TalonFXFactory.createTalon(
    Constants.ExampleFlywheel.motorID,
    Constants.ExampleFlywheel.motorCANBus,
    Constants.ExampleFlywheel.configuration);

  //Initialize Motion Magic velocity control
  private final MotionMagicVelocityVoltage m_motionMagicControl =
      new MotionMagicVelocityVoltage(0);
   
  //Initalize target velocity variable to minimum value
  private double m_targetVelocity = Constants.ExampleFlywheel.minTargetVelocity;

  //Method to report current flywheel velocity in ROTATIONS per SECOND
  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  //Method to assign target flywheel velocity in ROTATIONS per SECOND
  public void setVelocity(double targetVelocity) {
    m_targetVelocity = targetVelocity;
  }

  @Override
  public void periodic() {
    //Periodically sets desired flywheel velocity as a Motion Magic setpoint
    setMotionMagicVelocitySetpoint(
        Constants.ExampleFlywheel.motorPositionSlot,
        m_targetVelocity,
        0.0);
    //Displays flywheel current and target velocities on SmartDashboard
    SmartDashboard.putNumber("Flywheel/Current Velocity", m_motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Target Velocity", m_motor.getClosedLoopReference().getValueAsDouble());
  }
  
  //Method to set velocity (and slot, feedforward) periodically as a motion magic setpoint
  public void setMotionMagicVelocitySetpoint(
      final int slot,
      final double setpoint,
      final double feedforwardVolts) {
    m_motionMagicControl.Slot = slot;
    m_motionMagicControl.Velocity = setpoint;
    m_motionMagicControl.FeedForward = feedforwardVolts;
    m_motor.setControl(m_motionMagicControl);
  }

}
