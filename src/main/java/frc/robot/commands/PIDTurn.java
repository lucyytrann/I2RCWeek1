
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class PIDTurn extends CommandBase{
  /** Creates a new PIDTurn. */
   DriveTrain dt; 
   Double setpointAngle;
  PIDController pid = new PIDController(Constants.PIDConstants.PIDConstant, 0, 0);
  Double motorSign; 
  Double output;

  //take constants and created calculations 
  public PIDTurn(DriveTrain dt, double setpointAngle) {
    this.dt = dt;
    this.setpointAngle = setpointAngle;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(dt);
    pid.setTolerance(1.0);
    
    if(setpointAngle >= 0){
      motorSign = 1.0;
    } else{
      motorSign = -1.0;
    }
}
  //Called when the command is initially scheduled 
  public void initialize(){
    dt.resetNavx();
    dt.tankDrive(0, 0);
  }
  
  //Called every time the scheduler runs while the command is schdueled 
  @Override
  public void execute() {
    //(left, right) - turns CCW or in the position direction according 
     output = pid.calculate(dt.getAngle(), setpointAngle);
    dt.tankDrive(motorSign * output, -motorSign * output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
    //If pid is at the set point then it will finish
    //return Math.abs(dt.getAngle()) >= Math.abs(setpointAngle);
  }
}