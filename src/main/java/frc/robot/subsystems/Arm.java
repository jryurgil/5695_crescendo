// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_ArmMotor;
  private SparkPIDController armPID;
  private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  private GenericEntry newP = tab.add("newP", 0).getEntry();
private GenericEntry target = tab.add("target", 0).getEntry();

  /** Creates a new ExampleSubsystem. */
  public Arm() {

    m_ArmMotor = new CANSparkMax(6,MotorType.kBrushless);
    m_ArmMotor.getEncoder().setPosition(0);
    armPID = m_ArmMotor.getPIDController();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public double armPosition() {
    // Query some boolean state, such as a digital sensor.
    return m_ArmMotor.getEncoder().getPosition();
  }

  public void setArmPower(double power){
    
    if(m_ArmMotor.getEncoder().getPosition()<0){
      power=Math.max(0,power);
    }else if(m_ArmMotor.getEncoder().getPosition()>100){
      power=Math.min(0,power);
    }
    m_ArmMotor.set(power);
  }

public void setArmTarget(double target){
armPID.setReference(target,CANSparkBase.ControlType.kPosition);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", m_ArmMotor.getEncoder().getPosition());
     SmartDashboard.putNumber("arm P",armPID.getP());
       SmartDashboard.putNumber("arm I",armPID.getI());
      armPID.setP(newP.getDouble(0));
     // armPID.setReference(target.getDouble(0),CANSparkBase.ControlType.kPosition);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
