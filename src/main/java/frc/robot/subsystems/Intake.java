// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intakeIn;

public class Intake extends SubsystemBase {

  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_intakeLifter;
  private SparkPIDController intakePID;
DigitalInput notedetector;
//private ShuffleboardTab tab = Shuffleboard.getTab("Smart Dashboard");
//private GenericEntry newP = tab.add("newP", 0).getEntry();
//private GenericEntry target = tab.add("target", 0).getEntry();
  /** Creates a new ExampleSubsystem. */
  public Intake() {

    m_intakeMotor = new CANSparkMax(3,MotorType.kBrushless);
    m_intakeLifter = new CANSparkMax(2, MotorType.kBrushless);
    m_intakeMotor.getEncoder().setPosition(0);
    m_intakeLifter.getEncoder().setPosition(0);
    intakePID = m_intakeLifter.getPIDController();
     intakePID.setP(0.75);
    intakePID.setReference(0, ControlType.kPosition);
    notedetector = new DigitalInput(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public double intakePosition() {
    // Query some boolean state, such as a digital sensor.
    return m_intakeLifter.getEncoder().getPosition();
  }

  public void setIntakeSpeed(double power){
    m_intakeMotor.set(power);

  }
//starting with the intake up as 0, the down position is positive 30
  public void setIntakePosition(double position){
//check position for valid range
if (position <-35){
  position =-35;
} else if (position>0){
  position=0;
}
    intakePID.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("intake target", position);
  }

  public boolean hasNote(){
    //detector reads true when not blocked, false when not is in place
    //method returns true when a note is in the beam
    return !notedetector.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake encoder", m_intakeLifter.getEncoder().getPosition());
   SmartDashboard.putNumber("intake P", intakePID.getP());
     SmartDashboard.putBoolean("note detector", m_intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());
     //intakePID.setP(newP.getDouble(0));
     
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
