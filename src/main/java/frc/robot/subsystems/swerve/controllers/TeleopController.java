package frc.robot.subsystems.swerve.controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController {
  
  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;

  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega) {
    this.controllerX = controllerX; 
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
  }

  public ChassisSpeeds update() {

  }
}
