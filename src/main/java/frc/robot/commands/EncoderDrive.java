// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class EncoderDrive extends CommandBase {
  /** Creates a new EncoderDrive. */
  DriveTrain dt;
  Double setpoint;

  // It could pass to the drive train
  // It could customize the distance of robot
  public EncoderDrive(DriveTrain dt, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.setpoint = setpoint;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  // Run once
  @Override
  public void initialize() {
    dt.resetEncoders();
    // measure the distance
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Keep running until you tell it stop
  @Override
  public void execute() {
    // While the setPoint is greater than the ticks, then robot keep moving
    while (setpoint > dt.ticksToMeters()) {
      dt.tankDrive(0.2, 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  // If the setpoint less than the ticks(distance), then it keep moving, which is 'false' finished (not finished)
  // Otherwise, it stop
  public boolean isFinished() {
    return dt.ticksToMeters() >= setpoint;
  }
}
