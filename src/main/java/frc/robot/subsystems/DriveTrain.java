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

    //Set true to inverted of leftDriveTalon, set false to inverted of rightDriveTalan
    leftDriveTalon.setInverted(true);
    rightDriveTalon.setInverted(false);

    //Set true to sensor phase of both leftDriveTalon and rightDriveTalon
    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);
    
    //Config the Factory Defalt of the leftDriveTalon
    leftDriveTalon.configFactoryDefault();
    //Config the selected feedback sensor of the leftDriveTalon
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //Config the factory defalt of the rightDriveTalon
    rightDriveTalon.configFactoryDefault();
    //Config the selected feedback sensor of the rightDriveTalon
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }
  //Set the rightSpeed of the EightDriveTalon and set the leftSpeed of the leftDriveTalon
  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  //Reset tthe encoders 
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  //Return the TIcks by calculated the average of selected sensor position of leftDriveTalon and rightDriveTalon
  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
  //Get angle of navx
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  //Reset the navx
  public void resetNavx(){
    navx.reset();
  }

  //Override the periodic by using SmartDashboard value 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}