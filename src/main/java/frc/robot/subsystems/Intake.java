// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intakeIn;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_intakeMotor;
DigitalInput notedetector;
  /** Creates a new ExampleSubsystem. */
  public Intake() {

    m_intakeMotor = new CANSparkMax(20,MotorType.kBrushless);
    m_intakeMotor.getEncoder().setPosition(0);
notedetector = new DigitalInput(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public double intakePosition() {
    // Query some boolean state, such as a digital sensor.
    return m_intakeMotor.getEncoder().getPosition();
  }

  public void setIntakeSpeed(double power){
    m_intakeMotor.set(power);

  }

  public boolean hasNote(){
    //detector reads true when not blocked, false when not is in place
    //method returns true when a note is in the beam
    return !notedetector.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake encoder", m_intakeMotor.getEncoder().getPosition());
     SmartDashboard.putBoolean("note detector", notedetector.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
