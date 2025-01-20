package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.util.TalonFXFactory;

public class CoralIntakeSubsystem extends SubsystemBase {
 /* This is an example Subsystem for a open-loop single-motor mechanism.
  * Examples include Intake, Feeder, and Conveyer Rollers.
  *
  * This example uses TALON FX Duty Cycle Output in terms of power proportion (0.0 to 1.0).
  * Example constants for a Roller are in Constants.java.
  *
  *https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/DutyCycleOut.html
  *
  * Note: DutyCycleOut (0-1 proportion) produces an output with a percentage of available voltage, whereas
  * VoltageOut (1-12 Volts) produces a consistent voltage output. If you care about a roller performing
  * with a similar energy output regardless of battery capacity, then VoltageOut may be preferable
  * but may result in issues when the battery does not have an adequate capacity and may also be
  * less battery-efficient.
  *
  *https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
  */
  
  
  
    //Create motor using TalonFXFactory
    private TalonFX m_motor = TalonFXFactory.createTalon(
      Constants.Intake.motorID,
      Constants.Intake.motorCANBus,
      Constants.Intake.configuration);
  
       private TimeOfFlight intakeSensor = new TimeOfFlight(Intake.intakeSensorID);

       public CoralIntakeSubsystem() {

        intakeSensor.setRangingMode(Intake.intakeSensorRange, Intake.intakeSampleTime);
        intakeSensor.setRangeOfInterest(8, 8, 12, 12);

       }

  /**************************** DUTY CYCLE ****************************/
  //Initialize Duty Cycle control
  private final DutyCycleOut m_dutyCycleControl =
      new DutyCycleOut(0.0);
   
  //Initalize target output variable to minimum output
  private double m_dutyCycle = Constants.Intake.minOutput;

  //Method to report current roller applied Duty Cycle
  public double getDutyCycle() {
    return m_motor.getDutyCycle().getValueAsDouble();
  }

  //Method to assign target Duty Cycle
  public void setDutyCycle(double dutyCycle) {
    m_dutyCycle = dutyCycle;
  }

  //Method to set duty cycle periodically
  public void setDutyCycleSetpoint(final double setpoint) {
    m_dutyCycleControl.Output = setpoint;
    m_motor.setControl(m_dutyCycleControl);
  }

  /**************************** VOLTAGE ****************************/
  //Initialize Voltage control
  private final VoltageOut m_voltageControl =
      new VoltageOut(0.0);
   
  //Initalize target output variable to minimum voltage output
  private double m_voltage = Constants.Intake.minVoltage;

  //Method to report current roller applied voltage (-12 to 12 assuming battery is full) in Volts
  public double getVoltage() {
    return m_motor.getMotorVoltage().getValueAsDouble();
  }

  //Method to assign target voltage
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

   //Method to set voltage periodically
  public void setVoltageSetpoint(final double setpoint) {
    m_voltageControl.Output= setpoint;
    m_motor.setControl(m_voltageControl);
  }

  public boolean isCoralPresentTOF() {
    return intakeSensor.getRange() < Intake.isNotePresentTOF;
  }

  public boolean isCoralCenteredTOF() {
    return intakeSensor.getRange() != 0.0 && intakeSensor.getRange() < 100;
  }

  public double getRangeTOF() {
    return intakeSensor.getRange();
  }

  @Override
  public void periodic() {
    //COMMENT ONE
    //Periodically sets desired duty cycle output setpoint
    setDutyCycleSetpoint(m_dutyCycle);
    //Periodically sets desired voltage output setpoint
    setVoltageSetpoint(m_voltage);
    //Displays roller duty cycle and voltage on SmartDashboard
    SmartDashboard.putNumber("Roller/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Roller/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
  }

}