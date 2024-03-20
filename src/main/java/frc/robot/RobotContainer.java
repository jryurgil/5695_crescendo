// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//added comment on John's laptop
//added comment on cat laptop twice twice
//added comment on 'we can build it laptop' twice
//2nd comment
//added comment on no sticker laptop twice
//comment from Saturday jan 27
//comment 
//comment 2
//comment 3
//comment 4
//comment 5
//comment 6
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmtoAmp;
import frc.robot.commands.ArmtoGround;
import frc.robot.commands.intakeExpel;
import frc.robot.commands.intakeExpeltimed;
import frc.robot.commands.intakeIn;
import frc.robot.commands.targetFollow;
import frc.robot.commands.armRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake robotintake = new Intake();
  private final Arm robotarm = new Arm();
  private final Lifter robotlifter = new Lifter();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController2 = new CommandXboxController(1);
  //autonomous option chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    //populating the autonomous option list
    m_chooser.setDefaultOption("forward", movementonly());
    m_chooser.addOption("blue", blueauto());
     m_chooser.addOption("red", redauto());
    //m_chooser.addOption("tagfollower", tagfollower());

    SmartDashboard.putData(m_chooser);
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()/2, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()/2, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()/2, OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));

            robotarm.setDefaultCommand(new RunCommand(()-> robotarm.setArmTarget(robotarm.armPosition()+m_driverController2.getRightY()), robotarm));
            robotintake.setDefaultCommand(new RunCommand(()-> robotintake.setIntakePosition(robotintake.intakePosition()+m_driverController2.getLeftX()), robotintake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   
            m_driverController.x().whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            m_driverController2.rightBumper().whileTrue(new intakeIn(robotintake));
            m_driverController2.leftBumper().whileTrue(new intakeExpel(robotintake));
            m_driverController2.povLeft().whileTrue(new RunCommand(()-> robotintake.setIntakePosition(robotintake.intakePosition()-1), robotintake));
             m_driverController2.povRight().whileTrue(new RunCommand(()-> robotintake.setIntakePosition(robotintake.intakePosition()+1), robotintake));
             m_driverController2.povUp().whileTrue(new RunCommand(()-> robotarm.setArmTarget(robotarm.armPosition()-1), robotarm));
             m_driverController2.povDown().whileTrue(new RunCommand(()-> robotarm.setArmTarget(robotarm.armPosition()+1), robotarm));
             m_driverController.a().whileTrue(new RunCommand(()-> robotlifter.setLifterTarget(robotlifter.lifterPosition()+1),robotlifter));
             m_driverController.b().whileTrue(new RunCommand(()-> robotlifter.setLifterTarget(robotlifter.lifterPosition()-1),robotlifter));
            new Trigger(m_driverController2.y()).onTrue(new ArmtoAmp(robotarm));
            new Trigger(m_driverController2.x()).onTrue(new ArmtoGround(robotarm));
          
  }
  
public void resetTargets(){
    //reset the targets to match their current position
    robotintake.setIntakePosition(robotintake.intakePosition());
    robotarm.setArmTarget(robotarm.armPosition());
    robotlifter.setLifterTarget(robotlifter.lifterPosition());
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command redauto() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
       1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0, 0.3), new Translation2d(0, 0.6)),
        // End 3 meters straight ahead of where we started, facing forward
        //positive rotation is left turn
        new Pose2d(0, 1, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

public Command blueauto() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
       1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory bluestarttoamp = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
  
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.5, 0.45, new Rotation2d(0))),
        // End 3 meters straight ahead of where we started, facing forward
        //positive rotation is left turn
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand bluetoampcommand = new SwerveControllerCommand(
        bluestarttoamp,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        Trajectory amptofield = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
    
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.5, -0.5, new Rotation2d(0))),
          // End 3 meters straight ahead of where we started, facing forward
          //positive rotation is left turn
          config);
  
     
      SwerveControllerCommand amptofieldcommand = new SwerveControllerCommand(
          amptofield,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,
  
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
          
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(bluestarttoamp.getInitialPose());

    // Run path following command, then stop at the end.
    return bluetoampcommand.andThen(new armRelease(robotlifter)).andThen(new intakeExpeltimed(robotintake)).andThen(amptofieldcommand).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public Command movementonly() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
       1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        //positive rotation is left turn
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  //this program follows apriltag
  public Command tagfollower() {
   // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    return new targetFollow(m_robotDrive);
  }


  
}
