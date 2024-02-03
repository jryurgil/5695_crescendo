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
            double ty = LimelightHelpers.getTY("");
            double ta = LimelightHelpers.getTA("");
            boolean tv = LimelightHelpers.getTV("");
            double[] targetpose =LimelightHelpers.getTargetPose_CameraSpace("");
            //double targetangle = targetpose[5];
            double rot = -tx/50;//25 is the limit view
            double xSpeed = -(ta-6)/20;//ta=5 is the target, 0.2 is the speed limit
            double ySpeed = targetpose[4]/200;//- move right
            if (ta<5){ySpeed = 0;}
            if (xSpeed>0.2) {xSpeed=0.2;}
            else if (xSpeed<-0.2) {xSpeed=-0.2;}
            if (!tv) {xSpeed = 0;rot = 0; ySpeed =0;}
            
            driveSubsystem.drive(xSpeed, ySpeed, rot, false, true);
            SmartDashboard.putNumber("tx", LimelightHelpers.getTX(""));
            SmartDashboard.putNumber("ty",LimelightHelpers.getTY(""));
            SmartDashboard.putNumber("rot",rot);
            if(tv){
            SmartDashboard.putNumber("targetpose0", targetpose[0]);
            SmartDashboard.putNumber("targetpose1", targetpose[1]);
            SmartDashboard.putNumber("targetpose2", targetpose[2]);
            SmartDashboard.putNumber("targetpose3", targetpose[3]);
            SmartDashboard.putNumber("targetpose4", targetpose[4]);
            SmartDashboard.putNumber("targetpose5", targetpose[5]);
            }

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
