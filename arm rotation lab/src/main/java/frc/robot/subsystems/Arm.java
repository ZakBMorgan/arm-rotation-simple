// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public PIDController PID = new PIDController(
    Constants.PIDConstants.armPID_P,
    Constants.PIDConstants.armPID_I,
    Constants.PIDConstants.armPID_D);
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
