package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class targetFollow extends Command{
    private final DriveSubsystem driveSubsystem;

    public targetFollow(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("targetFollow cmd started");
    }

    @Override
    public void execute(){
            double tx = LimelightHelpers.getTX("");
            double ty= LimelightHelpers.getTY("");
            double rot;
                if (tx>5) {
                //turnright
                rot = -0.1;
            }else if (tx<-5){
                //turn left
                rot = 0.1;
            }else{
                //stay still
                rot = 0;
            }
            driveSubsystem.drive(0, 0, rot, true, true);
            SmartDashboard.putNumber("tx", LimelightHelpers.getTX(""));
            SmartDashboard.putNumber("ty",LimelightHelpers.getTY(""));
            SmartDashboard.putNumber("rot",rot);
            SmartDashboard.updateValues();
              

    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
