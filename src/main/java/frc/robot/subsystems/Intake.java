// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
  TalonSRX intakeMotor = new TalonSRX (14);

  public Intake() {

    intakeMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void setSpeed (double speed) {

    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);

  }

  @Override public void periodic () {
    setSpeed(0.1);
    // This method will be called once per scheduler run
  }
}