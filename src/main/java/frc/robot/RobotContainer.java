package frc.robot;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SpeakerShoot;
import frc.robot.commands.AShoot;
import frc.robot.commands.CornerAShoot;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.NoteCentricDrive;
import frc.robot.commands.PassingShot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArmAim;
import frc.robot.commands.SetArmAimAuto;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.SpeakerFieldRelativeDrive;
import frc.robot.commands.AutoIntakeNote;
import frc.robot.commands.AutoSpeakerShoot;
import frc.robot.commands.AutoBeamBreak;

import frc.robot.commands.AmpShoot;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BottomShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopShooter;
import frc.robot.subsystems.ClimbingSubsystem;
//import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {
    GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
    //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    //XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    SendableChooser<Command> m_chooser;
    private final Arm m_arm = new Arm();
    public final TopShooter m_ShooterSubsystem = new TopShooter();
    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public final BottomShooter m_bottom = new BottomShooter();
    public final ClimbingSubsystem m_climbingSubsystem = new ClimbingSubsystem();
    public Command centerPathCommand;
    public Command speakerPathCommand;
    public Command stageBottomPathCommand;
    public Command stageMiddleCommand;
    public Command stageTopPathCommand;
    public Command AmpPathCommand;
    PIDController followPID = new PIDController(.02, 0, 0);

    public RobotContainer() {// Configure the button bindings
        //m_robotDrive.AutonomousBuilder();
        //Subsystem Initialization Functions
        m_arm.armInitialize();
        m_ShooterSubsystem.enable();
        m_bottom.enable();
        m_arm.enable();
        buildPathCommands();
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    
        configureButtonBindings();
        m_robotDrive.calibrateGyro();
        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new FieldCentricDrive(m_robotDrive, m_driverController));
            //new RunCommand(
            //    () -> m_ShooterSubsystem.runFeeder(m_driverController.getRightTriggerAxis()), m_ShooterSubsystem);
        // m_climbingSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         () -> m_climbingSubsystem.SetClimbSpeed(-m_driverController.getRawAxis(2) + m_driverController.getRawAxis(3)), m_climbingSubsystem));    
            

        NamedCommands.registerCommand("Intake", new AutoIntakeNote(m_arm, m_intakeSubsystem));//new IntakeNote(m_arm, m_intakeSubsystem));
        NamedCommands.registerCommand("SubwooferShoot", new SpeakerShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("AmpShoot", new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("AShoot", new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("CornerAShoot", new CornerAShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));

        //Efficient Commands
        NamedCommands.registerCommand("BeamBreak", new AutoBeamBreak(m_intakeSubsystem));
        NamedCommands.registerCommand("SetArm", new SetArm(m_arm, 55));//Adjust this, might be different for each shoot pos.
        NamedCommands.registerCommand("SetArmAim", new SetArmAimAuto(m_arm, m_robotDrive.vision.LocationToSpeaker(), m_ShooterSubsystem, m_bottom));
        NamedCommands.registerCommand("AutoSpeakerShoot", new AutoSpeakerShoot(m_intakeSubsystem, m_ShooterSubsystem, m_bottom));
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_chooser);
        
    }
    
    //private GenericEntry setPoint = m_arm.tab.add("setPoint", 90).getEntry();
    private void configureButtonBindings() { //NOTE: All button commands
         
        // new JoystickButton(m_driverController, Button.kX.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        //new JoystickButton(m_driverController, Button.kA.value).onTrue(new RunCommand(() -> m_arm.setGoal(setPoint.getDouble(0)), m_arm));
        //new JoystickButton(m_driverController, Button.kStart.value).toggleOnTrue(new IntakeNote(m_arm, m_intakeSubsystem));//new StartEndCommand(() -> m_ShooterSubsystem.runFeeder(.5),() -> m_ShooterSubsystem.runFeeder(0), m_ShooterSubsystem));
        


        // new JoystickButton(m_driverController, Button.kStart.value).whileTrue(new ParallelCommandGroup(new PassingShot(m_robotDrive, m_driverController, followPID), new SetArm(m_arm, 50), new SetShooterSpeed(m_ShooterSubsystem, m_bottom, 3500, 0, 0)));
        
        

        //new JoystickButton(m_driverController, Button.kBack.value).onTrue(new InitializePrepareShoot(ArmAdjustmentActiveTF, this, m_ShooterSubsystem, m_bottom));
        // new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new ParallelCommandGroup(new SetArm(m_arm, 115), AmpPathCommand).andThen(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        // new JoystickButton(m_driverController, Button.kY.value).onTrue(new RunCommand(() -> m_arm.setGoal(115), m_arm));
        //new JoystickButton(m_driverController, Button.kB.value).toggleOnTrue(stageBottomPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kY.value).toggleOnTrue(stageMiddleCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        // new JoystickButton(m_driverController, Button.kA.value).toggleOnTrue(stageTopPathCommand.andThen(new AShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)));
        //new JoystickButton(m_driverController, Button.kRightBumper.value).toggleOnTrue(new AmpShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom));

        new JoystickButton(m_driverController, Button.kLeftBumper.value).toggleOnTrue(new ParallelRaceGroup(
            new NoteCentricDrive(m_robotDrive, m_driverController, followPID), 
            new IntakeNote(m_arm, m_intakeSubsystem, m_driverController)));
        new POVButton(m_driverController, 90).onTrue(new SpeakerShoot(m_arm, m_intakeSubsystem, m_ShooterSubsystem, m_bottom)
            );
        new POVButton(m_driverController, 270).whileTrue(new ParallelCommandGroup(
            new SetArmAim(m_arm, m_robotDrive, m_ShooterSubsystem, m_bottom), 
            new SpeakerFieldRelativeDrive(m_robotDrive, m_driverController, followPID)).andThen(new FieldCentricDrive(m_robotDrive, m_driverController)));
        new JoystickButton(m_driverController, Button.kRightBumper.value).whileTrue(new StartEndCommand(() -> m_intakeSubsystem.setSpeed(1),
            () -> m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
    }

    
    

    //private GenericEntry kp = m_ShooterSubsystem.tab.add("kp", 0).getEntry();
    //private GenericEntry ki = m_ShooterSubsystem.tab.add("ki", 0).getEntry();
    //private GenericEntry kd = m_ShooterSubsystem.tab.add("kd", 0).getEntry();
    //private GenericEntry PID_kp = m_arm.tab.add("PID_kp", 0.5).getEntry();
    //private GenericEntry PID_ki = m_arm.tab.add("PID_ki", 0.5).getEntry();
    //private GenericEntry PID_kd = m_arm.tab.add("PID_kd", 0).getEntry();



    public void anglePublish(){
        SmartDashboard.putNumber("Arm Angle", m_arm.getMeasurement());
        SmartDashboard.putData(m_arm);
        //m_arm.getController().setPID(PID_kp.getDouble(0.5), PID_ki.getDouble(0.5
        //), PID_kd.getDouble(0));
        //m_ShooterSubsystem.getController().setPID(kp.getDouble(0), ki.getDouble(0), kd.getDouble(0));
    }

    private void buildPathCommands(){
        //Pathfinding commands, going to 3 set points. Prep for shooting a note from defined locations
        PathPlannerPath path = PathPlannerPath.fromPathFile("Cent");

        PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    
        centerPathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints, 0.0);

        
        //####End of Orig Commands

        
        //PathPlannerPath SpeakerPath = PathPlannerPath.fromPathFile("Shoot - At Speaker");
        //speakerPathCommand = AutoBuilder.pathfindThenFollowPath(SpeakerPath, constraints, 0.0);
        PathPlannerPath StageBottomPath = PathPlannerPath.fromPathFile("B");
        stageBottomPathCommand = AutoBuilder.pathfindThenFollowPath(StageBottomPath, constraints, 0.0);
        PathPlannerPath StageMiddlePath = PathPlannerPath.fromPathFile("Y");
        stageMiddleCommand = AutoBuilder.pathfindThenFollowPath(StageMiddlePath, constraints, 0.0);
        PathPlannerPath StageTopPath = PathPlannerPath.fromPathFile("X");
        stageTopPathCommand = AutoBuilder.pathfindThenFollowPath(StageTopPath, constraints, 0.0);
        PathPlannerPath AmpPath = PathPlannerPath.fromPathFile("RB");
        AmpPathCommand = AutoBuilder.pathfindThenFollowPath(AmpPath, constraints, 0.0);

    }
    
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    }

    Rotation2d speakerRotation = new Rotation2d(m_robotDrive.vision.LocationToSpeaker()[0], m_robotDrive.vision.LocationToSpeaker()[1]);
    

  public Optional<Rotation2d> getRotationTargetOverride(){
    // Some condition that should decide if we want to override rotation
    if(m_robotDrive.vision.Found7() && m_intakeSubsystem.sensor()) {
        // Return an optional containing the rotation override (this should be a field relative rotation)
        return Optional.of(speakerRotation);
    } else {
        // return an empty optional when we don't want to override the path's rotation
        return Optional.empty();
    }
    }
}
