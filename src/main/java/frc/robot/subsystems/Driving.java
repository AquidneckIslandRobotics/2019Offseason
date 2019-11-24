/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.commands.CheesyDrive2;
import com.ctre.phoenix.motorcontrol.ControlMode; 
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import com.ctre.phoenix.MotorType.can.TalonSRX; 
//import com.ctre.CANSparkMaxLowLevel.MotorType;


import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Driving extends Subsystem {
      WPI_TalonSRX leftLeader = new WPI_TalonSRX(RobotMap.leftLeader); 
      WPI_TalonSRX leftFollower = new WPI_TalonSRX(RobotMap.leftFollower);
      WPI_TalonSRX rightLeader = new WPI_TalonSRX(RobotMap.rightLeader);
      WPI_TalonSRX rightFollower = new WPI_TalonSRX(RobotMap.rightFollower);
      Encoder rightEncoder = new Encoder(RobotMap.encoderRA,RobotMap.encoderRB);
      Encoder leftEncoder = new Encoder(RobotMap.encoderLA,RobotMap.encoderLB);
     DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader); 

     
      public boolean drivingForwards = true;
    

public void cheesyDrive2() {
  diffDrive.curvatureDrive(-Robot.m_oi.getSpeed(), Robot.m_oi.getRotation(), Robot.m_oi.getQuickTurn());
  
}

public void inverseCheesyDrive2() {
diffDrive.curvatureDrive(Robot.m_oi.getSpeed(), Robot.m_oi.getRotation(), Robot.m_oi.getQuickTurn());
}


public double getRightEncoder(){
  return rightEncoder.get();
}

public double getLeftEncoder(){
  return leftEncoder.get();
}
public void startDriveMotors() {
  leftLeader.set(0.5); 
  rightLeader.set(-0.5); 
}

public void stopDriveMotors() {
  leftLeader.set(0);
  rightLeader.set(0);
}
public void driveDistance(double distance){

}
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);


  // Called just before this Command runs the first time
  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive2()); 
  }
 //set followers
 public void initDriveControllers() {
  leftFollower.follow(leftLeader);
  rightFollower.follow(rightLeader); 
} 

  }

