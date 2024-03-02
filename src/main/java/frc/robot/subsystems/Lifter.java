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

public class Lifter extends SubsystemBase {

  private final CANSparkMax m_ArmMotor;
  private final CANSparkMax m_ArmMotor2;
  private SparkPIDController armPID;
   private SparkPIDController armPID2;
  private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  private GenericEntry newP = tab.add("newliftP", 0).getEntry();
//private GenericEntry target = tab.add("armtarget", 0).getEntry();

  /** Creates a new ExampleSubsystem. */
  public Lifter() {

    m_ArmMotor = new CANSparkMax(21,MotorType.kBrushless);
     m_ArmMotor2 = new CANSparkMax(20,MotorType.kBrushless);
    // m_ArmMotor2.follow(m_ArmMotor, true);
    m_ArmMotor.getEncoder().setPosition(0);
     m_ArmMotor2.getEncoder().setPosition(0);
    armPID = m_ArmMotor.getPIDController();
    armPID.setReference(0,CANSparkBase.ControlType.kPosition);
     armPID2 = m_ArmMotor2.getPIDController();
    armPID2.setReference(0,CANSparkBase.ControlType.kPosition);
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
  public double lifterPosition() {
    // Query some boolean state, such as a digital sensor.
    return m_ArmMotor.getEncoder().getPosition();
  }

  public void setLifterPower(double power){
    
    if(m_ArmMotor.getEncoder().getPosition()<0){
      power=Math.max(0,power);
    }else if(m_ArmMotor.getEncoder().getPosition()>100){
      power=Math.min(0,power);
    }
    m_ArmMotor.set(power);
  }

public void setLifterTarget(double target){
 /*  if (target>0){
    target=0;
  }else if (target<-70){
    target=-70;
  } */
armPID.setReference(target,CANSparkBase.ControlType.kPosition);
armPID2.setReference(-target,CANSparkBase.ControlType.kPosition);
SmartDashboard.putNumber("lift target", target);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lift encoder", m_ArmMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("lift encoder 2", m_ArmMotor2.getEncoder().getPosition());
     SmartDashboard.putNumber("lift P",armPID.getP());
       SmartDashboard.putNumber("lift I",armPID.getI());
      armPID.setP(newP.getDouble(0));
      armPID2.setP(newP.getDouble(0));
     // armPID.setReference(target.getDouble(0),CANSparkBase.ControlType.kPosition);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
