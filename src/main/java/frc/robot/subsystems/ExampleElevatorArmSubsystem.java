package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFXFactory;

public class ExampleElevatorArmSubsystem extends SubsystemBase {
 /* This is an example Subsystem for a position-controlled single-motor mechanism with an
  * integrated relative encoder. Examples include Elevators, Arms, Wrists, Pivots, and Turrets.
  *
  * This example uses TALON FX Motion-Magic Voltage Position-Control with units in motor
  * ROTATIONS. Example constants for both an Elevator and an Arm are in Constants.java.
  *
  *https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
  *
  * For a physical implementation it would be wise to find gear ratios and convert from rotations
  * to a measureable unit like inches or centimeters (for Elevators), 
  * or degrees or radians (for Arms, Wrists, Pivots, and Turrets). 
  *
  * An absolute encoder (CTRE Cancoder, REV through-bore) may also be useful for absolute precision cases, 
  * eliminating the need to re-zero physically before startup.
  */

  //Create motor using TalonFXFactory
  private TalonFX m_motor = TalonFXFactory.createTalon(
    Constants.ExampleElevator.motorID,
    Constants.ExampleElevator.motorCANBus, 
    Constants.ExampleElevator.configuration);

  //Initialize Motion Magic position control
  private final MotionMagicVoltage m_motionMagicControl =
      new MotionMagicVoltage(0);
   
  //Initalize target height variable to minimum height
  private double m_targetHeight = Constants.ExampleElevator.minHeight;

  //Method to report current height in ROTATIONS
  public double getHeight() {
    return m_motor.getPosition().getValueAsDouble();
  }

  //Method to assign target height in ROTATIONS
  public void setHeight(double targetHeight) {
    m_targetHeight = targetHeight;
  }

  @Override
  public void periodic() {
    //Periodically sets desired height as a Motion Magic setpoint
    setMotionMagicPositionSetpoint(
        Constants.ExampleElevator.motorPositionSlot,
        m_targetHeight,
        0.0);
    //Displays elevator current and target heights on SmartDashboard
    SmartDashboard.putNumber("Elevator/Current Height", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Target Height", m_motor.getClosedLoopReference().getValueAsDouble());
  }

  //Method to set height (and slot, feedforward) periodically as a motion magic setpoint
  public void setMotionMagicPositionSetpoint(
      final int slot,
      final double setpoint,
      final double feedforwardVolts) {
    m_motionMagicControl.Slot = slot;
    m_motionMagicControl.Position = setpoint;
    m_motionMagicControl.FeedForward = feedforwardVolts;
    m_motor.setControl(m_motionMagicControl);
  }
  
}