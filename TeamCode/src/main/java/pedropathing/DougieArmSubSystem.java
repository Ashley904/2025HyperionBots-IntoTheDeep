// File: DougieArmSubSystem.java
package pedropathing;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;

public class DougieArmSubSystem extends CommandBase {

    private static final double verticalSlideKp = 0.0265;
    private static final double verticalSlideKi = 0.0;
    private static final double verticalSlideKd = 0.0004;
    private static final double verticalSlideKf = 0.05;
    private final static double verticalSlideTicksInDegrees = 3.96;
    double verticalSlideTargetPosition;



    private static final double horizontalSlideKp = 0.0125;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.0006;
    private static final double horizontalSlideKf = 0.25;
    final double horizontalSlideTicksInDegrees = 3.96;

    double horizontalSlideTargetPosition;
    double currentHorizontalSlidePosition;

    PIDController verticalSlidePIDController;
    PIDController horizontalSlidePIDController;
    boolean isVerticalSlideAtTarget = false;
    boolean isHorizontalSlideAtTarget = false;

    DcMotor verticalSlideLeft;
    DcMotor verticalSlideRight;
    DcMotor horizontalSlide;

    MotionProfiledServo verticalLeftServo;
    MotionProfiledServo verticalRightServo;
    MotionProfiledServo verticalControlServo;
    MotionProfiledServo verticalRotationServo;
    MotionProfiledServo verticalGripperServo;

    MotionProfiledServo horizontalLeftServo;
    MotionProfiledServo horizontalRightServo;
    MotionProfiledServo horizontalControlServo;
    MotionProfiledServo horizontalRotationServo;
    MotionProfiledServo horizontalGripperServo;

    public DougieArmSubSystem(HardwareMap hardwareMap){
        verticalSlidePIDController = new PIDController(verticalSlideKp, verticalSlideKi, verticalSlideKd);
        horizontalSlidePIDController = new PIDController(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);

        verticalSlideLeft = hardwareMap.get(DcMotor.class, "VerticalSlideLeft");
        verticalSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        verticalSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalSlideRight = hardwareMap.get(DcMotor.class, "VerticalSlideRight");
        verticalSlideRight.setDirection(DcMotor.Direction.FORWARD);
        verticalSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlide = hardwareMap.get(DcMotor.class, "HorizontalSlide");
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeftServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalLeftServo"), 0.85, true);
        verticalRightServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalRightServo"), 0.85, false);
        verticalControlServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalControlServo"), 1.0, false);
        verticalGripperServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalGripperServo"), 1.0, false);
        verticalRotationServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalGripperRotation"), 1.0, false);

        horizontalLeftServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "horizontalLeftServo"), 0.785, true);
        horizontalRightServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "horizontalRightServo"), 0.785, false);
        horizontalControlServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "horizontalControlServo"), 1.0, false);
        horizontalGripperServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "horizontalGripperServo"), 1.0, true);
        horizontalRotationServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "horizontalGripperRotation"), 1.0, false);
    }

    /** Idle Positions **/
    public void OuttakeOpModeIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.375)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.375)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.375))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void OuttakeAutoIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(1)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.05)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.05))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }


    /** Specimen Actions **/
    public void PositionForSpecimenCollection(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 0),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.355)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.4)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.01065)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.01065))
                        )
                )
        );
    }

    public void PositionForSpecimenScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(300)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = -10),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.4215)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.4215)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.35))
                        ),

                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(100)),
                        new WaitUntilCommand(() -> verticalRightServo.isAtTarget(100)),

                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.55)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(225)),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625))
                )
        );
    }

    public void PositionForSpecimenSOpModeScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(100)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = -10),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.4215)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.4215)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.35))
                        ),

                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(100)),
                        new WaitUntilCommand(() -> verticalRightServo.isAtTarget(100)),

                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.55)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(225)),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625))
                )
        );
    }

    public void PositionToSlideSpecimensOnBar(){
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.49))
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> verticalSlideTargetPosition = 630),
                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),
                        new WaitCommand(250),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.3)),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void PositionForSampleScanning(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1050),
                        new InstantCommand(this:: LimeLightIntakeIdlePosition)
                )
        );
    }


    /** Sample Actions **/

    void IntakeOpModeIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.35)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),

                                new WaitCommand(500),

                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.575)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.575))
                        ),

                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget(50)),
                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget(50)),
                        new InstantCommand(() -> horizontalSlideTargetPosition = -100)
                )
        );
    }

    public void PositionForSampleCollection(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = 750),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.2175)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.2175)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.21)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24))
                        )
                )
        );
    }


    public void SampleCollectionModeCollectSample(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.168)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.168)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.23))
                        ),

                        new WaitCommand(150),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.705)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(400)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.23)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),

                                new WaitCommand(100),

                                new ParallelCommandGroup(
                                        new InstantCommand(() -> horizontalSlideTargetPosition = -65),
                                        new InstantCommand(() -> verticalSlideTargetPosition = 300),
                                        new WaitCommand(335),

                                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.35)),
                                        new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.285)),

                                        new WaitCommand(650),

                                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.5)),
                                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.5))
                                )
                        ),
                        new WaitCommand(100),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.8)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(320)),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.7))
                )
        );
    }

    public void SpecimenCollectionModeCollectSample(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.162)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.162)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.23))
                        ),

                        new WaitCommand(150),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.69)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(400)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.23)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),

                                new WaitCommand(100),
                                new InstantCommand(() -> horizontalSlideTargetPosition = -65)
                        )
                )
        );
    }

    void ThrowSampleOutIntoObservationZone(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalSlideTargetPosition = 1350),
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.24)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.24)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.3))
                        ),

                        new WaitCommand(200),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(50)),

                        new InstantCommand(this::IntakeOpModeIdlePosition)
                )
        );
    }

    public void TransferSampleToOuttake() {
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                // Intake Actions
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> horizontalSlideTargetPosition = 135),
                                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.23)),
                                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.6)),
                                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.6)),
                                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.4)),
                                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.6))
                                        ),
                                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget()),
                                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget())
                                ),

                                // Outtake Actions
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> verticalSlideTargetPosition = 400),
                                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),

                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.35)),
                                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.285)),

                                                new WaitCommand(200),

                                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.65)),
                                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.65))
                                        )
                                )
                        ),
                        new ParallelCommandGroup(

                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.665)),
                                new WaitCommand(100),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1))
                        ),

                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(165)),

                        new ParallelCommandGroup(

                                new InstantCommand(this::PositionForHighBucketScoring),
                                new InstantCommand(this::IntakeOpModeIdlePosition)
                        )
                )
        );
    }



    void PositionForHighBucketScoring(){

        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.65)),

                new ParallelCommandGroup(

                        new InstantCommand(() -> verticalSlideTargetPosition = 1900),
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.88)),

                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.285)),
                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.285)),

                        new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19))
                )
        );
    }

    void ScoreSampleInHighBasket(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget()),

                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.5)),
                        new WaitUntilCommand(() -> verticalControlServo.isAtTarget()),

                        new InstantCommand(() -> verticalSlideTargetPosition = 0),
                        new InstantCommand(this:: OuttakeOpModeIdlePosition)
                )
        );
    }

    /** Autonomous Sample Pick Up Actions **/
    public void AutonomousPositionForSampleCollection(int slideTargetPosition, double rotationServoTargetPosition){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = slideTargetPosition),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.2175)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.2175)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.21)),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(rotationServoTargetPosition))
                        )
                )
        );
    }



    /** Limelight Sample Pick Up Actions **/
    public void LimeLightIntakeIdlePosition() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.2))
                        ),

                        new WaitCommand(150),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.74))
                )
        );
    }



    void VerticalPIDFSlideControl() {
        verticalSlidePIDController.setPID(verticalSlideKp, verticalSlideKi, verticalSlideKd);

        int currentPosition = verticalSlideRight.getCurrentPosition();

        // Calculate error
        double error = verticalSlideTargetPosition - currentPosition;

        // Apply PID
        double pid = verticalSlidePIDController.calculate(currentPosition, verticalSlideTargetPosition);

        // Feedforward (gravity compensation)
        double feedforward = Math.cos(Math.toRadians(currentPosition / verticalSlideTicksInDegrees)) * verticalSlideKf;

        // Combine and limit power
        double power = pid + feedforward;
        power = Math.max(-1.0, Math.min(1.0, power));

        verticalSlideLeft.setPower(power);
        verticalSlideRight.setPower(power);

        isVerticalSlideAtTarget = Math.abs(error) < 50;
    }

    public void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);

        // Clamp target to stay within limits
        horizontalSlideTargetPosition = Math.max(0, Math.min(1800, horizontalSlideTargetPosition));

        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();
        double pid = horizontalSlidePIDController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalSlideTicksInDegrees)) * horizontalSlideKf;
        double adjustment = pid + feedForward;
        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));
        horizontalSlide.setPower(adjustment);

        isHorizontalSlideAtTarget = Math.abs(currentHorizontalSlidePosition - horizontalSlideTargetPosition) < 50;
    }
    public void updateServos() {
        verticalGripperServo.update();
        verticalLeftServo.update();
        verticalRightServo.update();
        verticalControlServo.update();
        verticalRotationServo.update();


        horizontalGripperServo.update();
        horizontalLeftServo.update();
        horizontalRightServo.update();
        horizontalControlServo.update();
        horizontalRotationServo.update();
    }
}