package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class FeedNote extends Command {

  private IntakeSubsystem intake;
   private TalonFX intakeTalon = new TalonFX(IntakeConstants.intakeTalonID);

  /** Creates a new IntakeNote. */
  public FeedNote(IntakeSubsystem intake){

    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTalon.set(.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOFF();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
