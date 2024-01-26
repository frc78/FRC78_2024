// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {

  Intake intaker;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake subsystem1) {
    // Use addRequirements() here to declare subsystem dependencies.
    intaker = subsystem1;
    addRequirements(intaker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intaker.setPIDReferenceBOTH(5000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intaker.setPIDReferenceBOTH(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
