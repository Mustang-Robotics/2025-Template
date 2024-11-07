package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopShooter;

public class AmpShoot extends SequentialCommandGroup {
    public AmpShoot(Arm arm, IntakeSubsystem intake, TopShooter top, BottomShooter bottom){
        addCommands(
            new SetShooterSpeed(top, bottom, 1500, 500, 150),
            new SetIntakeSpeed(intake, -1), //Load intake to shoot

            new WaitCommand(0.4),

            new SetIntakeSpeed(intake, 0), //Intake speed set to 0
            new SetShooterSpeed(top, bottom, 0, 0, 100), //Intake speed set to 0
            new SetArm(arm, 42) //Return arm to base angle
            );
    }

}

