package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.playingwithfusion.TimeOfFlight;

public class IntakeSubsystem extends SubsystemBase {
 
  public TalonFX intakeTalon = new TalonFX(IntakeConstants.intakeTalonID);
  private TimeOfFlight intakeSensor = new TimeOfFlight(4);
  private double intakerange;

  
  public IntakeSubsystem() {
    intakeSensor.setRangingMode(IntakeConstants.intakeSensorRange, IntakeConstants.intakeSampleTime);
    intakeSensor.setRangeOfInterest(8, 8, 12, 12);
  }

 public void intakeON() {
   intakerange = intakeSensor.getRange();
   if(intakerange > 254) {
     intakeTalon.set(.8);
   } else {
     intakeTalon.set(0);
   }
 
 }

  public void FeedCoral() {
    intakeTalon.set(.8);
  }

  public void intakeOFF() {
    intakeTalon.set(0);
  }

  public void outakeON() {
    intakeTalon.set(-.8);
  }

  public boolean isCoralPresentTOF() {
    return intakeSensor.getRange() < IntakeConstants.isCoralPresentTOF;
  }

  public boolean isCoralCenteredTOF() {
    return intakeSensor.getRange() != 0.0 && intakeSensor.getRange() < 100;
  }

  public double getRangeTOF() {
    return intakeSensor.getRange();
  }

  public boolean isIntakeRunning() {
    return Math.abs(intakeTalon.get()) > 0.01;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("range intake", getRangeTOF());
  }

}