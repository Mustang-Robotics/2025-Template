package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FieldCentricDrive extends Command{
    DriveSubsystem m_drive;
    GenericHID m_controller;

    public FieldCentricDrive(DriveSubsystem drive, GenericHID controller){
        m_drive = drive;
        m_controller = controller;

        addRequirements(m_drive);
    }

    @Override
    public void execute(){
        m_drive.drive(
                    -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_controller.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true, true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
