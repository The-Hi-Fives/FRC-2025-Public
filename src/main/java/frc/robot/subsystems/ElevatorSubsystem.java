package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ErrorCheckUtil;
import frc.robot.util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.util.TalonFXFactory;


public class ElevatorSubsystem extends SubsystemBase {

     //Elevator 1

  private TalonFX elevatorLeaderTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.motorID,
      ElevatorConstants.motorCANBus, ElevatorConstants.configuration));

     //Elevator 2

  private TalonFX elevatorFollowerTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.motorID2,
      ElevatorConstants.motorCANBus, ElevatorConstants.configuration));


  public ElevatorSubsystem() {
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move Elevator to position
   * 
   * @param height in meters (0 to max height)
   */
  public void setHeight(double height) {

    elevatorLeaderTalon.setControl(
      ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move elevator to home position (0)
   */
  public void stow() {
    setHeight(frc.robot.Constants.Setpoints.ElevatorStowHeight);
  }

  public void holdPosition() {

    elevatorLeaderTalon.setControl(new VoltageOut(ElevatorConstants.configuration.Slot0.kG));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Set all outputs to 0
   */
  public void stop() {

    elevatorLeaderTalon.setControl(new DutyCycleOut(0));
    elevatorFollowerTalon.setControl(new DutyCycleOut(0));
  }

  /**
   * Run elevator motors at a voltage
   * 
   * @param volts
   */
  public void setElevatorVoltage(double volts) {

    elevatorLeaderTalon.setControl(new VoltageOut(volts));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  public void resetEncoderPosition(double height) {

    elevatorLeaderTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
    elevatorFollowerTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderTalon.getClosedLoopError().getValue());
  }


  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorError", getSetpointError());
  }

    private TalonFX configureElevatorTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getStatorCurrent().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getReverseLimit().setUpdateFrequency(ElevatorConstants.kElevatorFastUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
  }
}
