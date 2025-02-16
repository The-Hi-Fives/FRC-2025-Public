// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist_Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.TalonFXFactory;

public class Wrist_Intake extends SubsystemBase {

  private TalonFX m_armLeader = TalonFXFactory.createTalon(WristConstants.armLeaderID, WristConstants.armTalonCANBus, WristConstants.kArmConfiguration);
  /** Creates a new Arm. */
  public Wrist_Intake() {
  }
   /**
   * PID arm to position
   * 
   * @param rotations 0 to 1 rotations
   */
  public void setAngle(double position) {
    m_armLeader.setControl(WristConstants.armPositionControl.withPosition(position));
  }

  /**
   * PID arm to position
   * 
   * @param rotations Rotation 2d
   */
  public void setAngle(Rotation2d position) {
    setAngle(position.getRotations());
  }

  public void stow() {
    setAngle(0);
  }

  /**
   * Just PID to the current angle to hold position
   */
 

  public Rotation2d getSetpointError() {
    return Rotation2d.fromRotations(m_armLeader.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError().getDegrees()) <= WristConstants.angleErrorTolerance.getDegrees();
  }

  public void stop() {
    m_armLeader.setControl(new DutyCycleOut(0));
  }

  public void setarmVoltage(double volts) {
    m_armLeader.setControl(new VoltageOut(volts));
  }


    public Command stowarmCommand() {
      return new RunCommand(()->this.stow(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getSetpointError().getDegrees());
  }
}
