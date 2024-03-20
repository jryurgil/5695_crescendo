// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmtoGround extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_arm;
//private final Intake m_intake;
private double armtarget;
//private double intaketarget;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmtoGround(Arm arm/* ,  Intake intake*/) {
    m_arm = arm;
   // m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
   // addRequirements (intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armtarget = m_arm.armPosition();
    //intaketarget = m_intake.intakePosition();
    m_arm.setArmTarget (armtarget);
    //m_intake.setIntakePosition (intaketarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armtarget <0){
    armtarget = Math.min(0, armtarget+1);
  }
  /* if (intaketarget <40){
    intaketarget++;
  } */
  
      m_arm.setArmTarget (armtarget);
      //m_intake.setIntakePosition (intaketarget);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (armtarget == 0 );
  }
}
