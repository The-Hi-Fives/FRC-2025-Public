package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.playingwithfusion.TimeOfFlight;

public class IntakeSubsystem extends SubsystemBase {
 
  public TalonFX intakeTalon = new TalonFX(IntakeConstants.intakeTalonID);
  private TimeOfFlight intakeSens = new TimeOfFlight(4);
  private double intakerange;

  
  public IntakeSubsystem() {
     
   
  }

  public void intakeON() {
    intakerange = intakeSens.getRange();

    if(intakerange > 254) {
      intakeTalon.set(.8);
    } else {
      intakeTalon.set(0);
    }
  
  }

  public void Feednote() {
    intakeTalon.set(.8);
  }

  public void intakeOFF() {
    intakeTalon.set(0);
  }

  public void outakeON() {
    intakeTalon.set(-.8);
  }

  // public void stop() {
  //   intakeTalon.setControl(IntakeConstants.intakeDutyCycle.withOutput(0));
  // }

  // public void setIntakeDutyCycle(double speed) {
  //   intakeTalon.setControl(IntakeConstants.intakeDutyCycle.withOutput(speed));
  // }

  // public void setIntakeTorqueControl(double amps) {
  //   intakeTalon.setControl(IntakeConstants.intakeTorqueControl.withOutput(amps));
  // }

  // public void setIntakeVoltage(double volts) {
  //   intakeTalon.setControl(new VoltageOut(volts));
  // }

  // public boolean isIntakeRunning() {
  //   return Math.abs(intakeTalon.get()) > 0.01;
  // }

  // @Override
  // public void periodic() {
  // }

  // private TalonFX configureIntakeTalon(TalonFX motor) {
  //   return motor;
  // }
}
