// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

  public PIDController PID = new PIDController(
    Constants.PIDConstants.armPID_P,
    Constants.PIDConstants.armPID_I,
    Constants.PIDConstants.armPID_D);

  //create Talon objects, for rotate and extension
  public static TalonSRX rotateTalon = new TalonSRX(Constants.OperatorConstants.armRotation);
  public static TalonSRX extensionTalon = new TalonSRX(Constants.OperatorConstants.armExtension);
  /** Creates a new Arm. */
  public Arm() {}

  public void setRot(double power) {
    rotateTalon.set(ControlMode.PercentOutput, power+(Math.sin(getAngle()))*(Constants.Measurements.armHoldingVoltage/12.0));
  }

  public void setExt(double power) {
    extensionTalon.set(ControlMode.PercentOutput, power);
  }

  public double getAngle() {
    return (rotateTalon.getSelectedSensorPosition(0)/4096*360);
  }

  public void resetEncoders() {
    rotateTalon.setSelectedSensorPosition(0, 0, 10);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setRot(RobotContainer.getJoy1().getY());
    
    if (RobotContainer.getJoy1().getPOV() == 0) {
      extensionTalon.set();
    }
  }
}