// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  //Decalre and initialize the variale 
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain");
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry();
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    //Intialized leftDriveTalon and rightDriveTalon
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    
    //Set neutral mode to both leftDriveTalon and right DriveTalon
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);
    
    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }
  //It is a tank drive that drives the motors 
  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  //Sensors that marks the ticks (position sensor)
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  // Count the ticks, gets position
  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
  public double ticksToMeters(){
    return (getTicks() * 0.1524 * Math.PI)/4096;
    //method is a function that you can reuse, and it have an input and ouput
  }

  //Get angle of the unit circle 
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  //Reset the angle to 0
  public void resetNavx(){
    navx.reset();
  }

  
  @Override
  public void periodic() {
    //Puts the data values that we want onto smartdashboard
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
    SmartDashboard.putNumber("Ticks", getTicks());
    //Move the ticks and and ticks
    //Encoder values 
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Meters Driven", ticksToMeters());
    
    //Set percent of the motor power
    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}
