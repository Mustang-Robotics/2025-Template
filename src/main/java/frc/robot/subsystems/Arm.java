
//SUMMARY NOTES: Declares the modules relating to the motors. (One Motor)
package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class Arm extends ProfiledPIDSubsystem{

    
    public static ShuffleboardTab tab = Shuffleboard.getTab("Arm"); //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    private GenericEntry forward_ks = tab.add("forward_ks", 0).getEntry();
    private GenericEntry forward_kg = tab.add("forward_kg", .5).getEntry();
    private GenericEntry forward_kv = tab.add("forward_kv", 0.07).getEntry();
    private GenericEntry forward_ka = tab.add("forward_ka", 0).getEntry();
    //private GenericEntry velocity = tab.add("velocity", 720).getEntry();
    //private GenericEntry acceleration = tab.add("acceleration", 720).getEntry();


    public final CANSparkMax armMotor = new CANSparkMax(16, MotorType.kBrushed); 

    //private final AnalogPotentiometer pot = new AnalogPotentiometer(0, 280, 0);
    //private final RelativeEncoder m_turningEncoder = armMotor.getAlternateEncoder(8192); //RevThroughBoreEncoder, how to initialize? //AbsoluteEncoder Not Relative
    public DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    
    private ArmFeedforward m_feedforward = new ArmFeedforward(forward_ks.getDouble(0), forward_kg.getDouble(0.5), forward_kv.getDouble(0.07), forward_ka.getDouble(0));
 

    public Arm(){
        super(
            new ProfiledPIDController(
            .5, 
            .5, 
            0, 
            new TrapezoidProfile.Constraints(
                180, 
                180)), 
                0);
        setGoal(42); //Startup arm angle 
        //encoderOffsetInRotations.setValue(armEncoder.getPositionOffset());
        //offsetToDegrees.setValue(getMeasurement());
        //aprilTagPoseTester();
    }

    // ##### Beginning of Main Functions #####
    public void armInitialize() { //This function is called via RobotContainer, since some variables need to be initialized.
        armEncoder.setDistancePerRotation(360);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint){
        //m_feedforward = new ArmFeedforward(forward_ks.getDouble(0), forward_kg.getDouble(0), forward_kv.getDouble(0), forward_ka.getDouble(0));
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        armMotor.setVoltage(-output - feedforward);
    }

    public void ArmMovementUp() { // NOTE: Starts the movement of the motor from 0-1. 1 is full, 0 is none.
        armMotor.set(0.3);
    }

    public void ArmMovementDown() {
        armMotor.set(0);
    }

    public void SetArmAngleIntake() {
        //setGoal(0);
    }

    public void TestSetArmAngleShoot(double angle) {
        setGoal(angle);// - 18); //Offset of 18 degrees
        
    }

    public void SetArmAngleShoot(double robotCurrentX) {
        setGoal(75);//getArmAngleFromTargetPosition(FieldPositionConstants.speakerPose.getX(), FieldPositionConstants.speakerPose.getY(), robotCurrentX));//speakerPose.getY(), speakerPose.getZ()));//targetX, targetY));
        
    }

    public void setEncoderPosition(){ //Sets the encoder distance traveled to 0. The new midpoint.
        //return m_turningEncoder.getVelocity();
        armEncoder.reset();
    }

    public void aprilTagPoseTester() {
        //AprilTagFieldLayout aprilTagFieldLayout;
        //aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        //System.out.print("Tag 7 Pos: " + aprilTagFieldLayout.getTagPose(7));
        //"Tag 7 Pos: " + 
    }
    // ##### End of Main Functions #####


    // ##### Beginning of Returns #####
    @Override
    public double getMeasurement(){ //Returns the physical distance (not angle) the encoder has traveled from the base value of 0
        //return m_turningEncoder.getVelocity();
        //var offset =  armEncoder.getPositionOffset();
        //return (offset / .002777);
        return armEncoder.getDistance();
        //return armEncoder.getDistancePerRotation();
    }

    public boolean getArmInRangeWithinLimit(double currentAngle, double desiredAngle, double upperLimit, double lowerLimit) {
        if((currentAngle <= desiredAngle + upperLimit) && (currentAngle >= desiredAngle - lowerLimit)) {
            return true;
        }else {
            return false;
        }
    }
    /*
    public double getArmAngleFromTargetPosition(double targetX, double targetY, double robotCurrentX) {
        //var uOffset = 0 - targetX; //Work Here
        var u = targetX; //Target X 
        var t = targetY; //Target Y 
        var r = ShooterConstants.armRadius;
        var v = robotCurrentX - ShooterConstants.armPivotXOffset; //robotArmPivotX --Current robot position - fixed distance to arm pivot 
        var w = ShooterConstants.armPivotZOffset; //robotArmPivotY --Current arm pivot height off of the ground.

        var a = Math.atan(t - w / u - v);
        var b = Math.acos(r / Math.sqrt(Math.pow(t - w, 2)) + Math.sqrt(Math.pow(u - v, 2)));
        var c = v + r * Math.cos(a + b); //Current arm X pos
        var d = w + r * Math.sin(a + b); //Current arm Y pos
        //var p = d - t / c - u; //Current arm shooting slope
        //Shooting line: y - d = p(x - c)
        var h = Math.sqrt(Math.pow((c-(v-18)), 2) + Math.pow((d - w), 2)); //Distance between Left bottom point on circle and arm point
        var q = Math.acos(2*Math.pow(r, 2) - Math.pow(h, 2)/2 * Math.pow(r, 2)); //Current arm angle
        System.out.print("------");
        System.out.print("Current X" + robotCurrentX);
        System.out.print("Current Angle" + q);

        //23 low and 115 high
        if (q < 23) {
            q = 23;
            System.out.print("Error: Angle Too Low");
        }else if (q > 115) {
            q = 115;
            System.out.print("Error: Angle Too High");
        }
        return q;
        
    }
     */
    /* 
    public double ArmAngle() {
        return pot.get();
    }*/
    // ##### End of Returns #####

    /* ##### Commands List #####

    new JoystickButton(m_driverController, Button.kB.value).onTrue(Commands.runOnce(()->{
        m_arm.setGoal(setPoint.getDouble(0));
        m_arm.enable();
    },
    m_arm));
    }

    */
}
