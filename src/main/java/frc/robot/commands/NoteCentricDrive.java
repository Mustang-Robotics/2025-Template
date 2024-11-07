package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class NoteCentricDrive extends Command{
    DriveSubsystem m_drive;
    GenericHID m_controller;
    PIDController m_PID;

    public NoteCentricDrive(DriveSubsystem drive, GenericHID controller, PIDController PID){
        m_drive = drive;
        m_controller = controller;
        m_PID = PID;

        addRequirements(m_drive);
    }

    @Override
    public void execute(){
        m_drive.drive(
                -MathUtil.applyDeadband(m_controller.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_controller.getRawAxis(0), OIConstants.kDriveDeadband),
                m_PID.calculate(m_drive.vision.rotationToNote(), 0)
                -MathUtil.applyDeadband(m_controller.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true, true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
