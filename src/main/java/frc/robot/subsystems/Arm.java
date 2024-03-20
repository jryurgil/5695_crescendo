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
  private final CANSparkMax m_ArmMotor2;
  private SparkPIDController armPID;

  //private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
 // private GenericEntry newP = tab.add("newarmP", 0).getEntry();
//private GenericEntry target = tab.add("armtarget", 0).getEntry();

  /** Creates a new ExampleSubsystem. */
  public Arm() {

    m_ArmMotor = new CANSparkMax(5,MotorType.kBrushless);
     m_ArmMotor2 = new CANSparkMax(6,MotorType.kBrushless);
     m_ArmMotor2.follow(m_ArmMotor, true);
    m_ArmMotor.getEncoder().setPosition(0);
    armPID = m_ArmMotor.getPIDController();
    armPID.setP(0.5);
    armPID.setReference(0,CANSparkBase.ControlType.kPosition);
    m_ArmMotor.setSmartCurrentLimit(80);
    m_ArmMotor2.setSmartCurrentLimit(80);
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
  if (target>0){
    target=0;
  }else if (target<-45){
    target=-45;
  }
armPID.setReference(target,CANSparkBase.ControlType.kPosition);
SmartDashboard.putNumber("arm target", target);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", m_ArmMotor.getEncoder().getPosition());
     SmartDashboard.putNumber("arm P",armPID.getP());
       SmartDashboard.putNumber("arm I",armPID.getI());
      //armPID.setP(newP.getDouble(0));
     // armPID.setReference(target.getDouble(0),CANSparkBase.ControlType.kPosition);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
