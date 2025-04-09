// File: DougieArmSubSystem.java
package pedropathing;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.2)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.45)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.45))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void OuttakeAutonomousInitIdlePosition(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.375)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.2)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.1)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.05)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.05))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = -15)
                )
        );
    }

    /** Specimen Actions **/
    public void PositionForSpecimenCollection(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 0),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.375)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.22)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.85)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.08)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.08))
                        )
                )
        );
    }

    public void PositionForSpecimenScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(100)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = 150),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.43)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.43)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.2)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.65))
                        )
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 840),
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

    void IntakeAutonomousInitIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),

                        new InstantCommand(() -> horizontalControlServo.setTargetPosition(1)),
                        new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.25)),

                        new WaitCommand(300),

                        new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.43)),
                        new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.43))
                )
        );
    }

    void IntakeOpModeIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.6)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.25)),

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
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.215)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.215)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.465)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.185))
                        )
                )
        );
    }


    public void SampleCollectionModeCollectSample(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.115)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.115)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.435))
                        ),

                        new WaitCommand(150),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.65)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(400)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.49)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),

                                new WaitCommand(100),
                                new InstantCommand(() -> horizontalSlideTargetPosition = -65)
                        ),
                        new WaitCommand(650),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.775)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(450)),

                       new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.7))
                )
        );
    }

    public void SpecimenCollectionModeCollectSample(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.115)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.115)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.46))
                        ),

                        new WaitCommand(150),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.65)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(400)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.49)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),

                                new WaitCommand(100),
                                new InstantCommand(() -> horizontalSlideTargetPosition = 0)
                        )
                )
        );
    }

    public void SlightlyDropSampleForTransfer(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.755)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(415)),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.7))
                )
        );
    }

    void ThrowSampleOutIntoObservationZone(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalSlideTargetPosition = 1350),
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.1)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.1)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.6))
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
                                                new InstantCommand(() -> horizontalSlideTargetPosition = 350),
                                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24)),
                                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.75)),
                                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.75)),
                                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.46)),
                                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.62))
                                        ),
                                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget()),
                                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget())
                                ),

                                // Outtake Actions
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> verticalSlideTargetPosition = 400),
                                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.22)),
                                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.535))
                                        ),
                                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),

                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.8)),
                                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.8))
                                        ),
                                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(0)),
                                        new WaitUntilCommand(() -> verticalRightServo.isAtTarget(0)),
                                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(0))
                                )
                        ),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.65)),
                                new WaitUntilCommand(() -> verticalGripperServo.isAtTarget()),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1)),

                                new ParallelCommandGroup(
                                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.3)),
                                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.3))
                                ),

                                new ParallelCommandGroup(
                                        new InstantCommand(this::PositionForHighBucketScoring),
                                        new InstantCommand(this::IntakeOpModeIdlePosition)
                                )
                        )
                )
        );
    }



    void PositionForHighBucketScoring(){

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.65)),

                        new InstantCommand(() -> verticalSlideTargetPosition = 2300),
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.88)),

                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.2)),
                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.2)),

                        new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.22))
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
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.185)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.185)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.43)),
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

    public void LimelightPositionForSampleCollection() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.185)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.185)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.44)),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(1))
                        )
                )
        );
    }

    public void LimelightCollectSample() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.13)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.13)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.46))
                        ),

                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget(25)),
                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget(25)),
                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(25)),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.74)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(100)),


                        new InstantCommand(this::LimeLightIntakeIdlePosition)
                )
        );
    }



    void VerticalPIDFSlideControl() {
        verticalSlidePIDController.setPID(verticalSlideKp, verticalSlideKi, verticalSlideKd);

        int currentVerticalSlidePosition = verticalSlideRight.getCurrentPosition();
        double PID = verticalSlidePIDController.calculate(currentVerticalSlidePosition, verticalSlideTargetPosition);

        double feedforward = Math.cos(Math.toRadians(verticalSlideTargetPosition / verticalSlideTicksInDegrees)) * verticalSlideKf;
        double adjustment = PID + feedforward;

        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);

        isVerticalSlideAtTarget = Math.abs(currentVerticalSlidePosition - verticalSlideTargetPosition) < 55;
    }
    public void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);

        // Clamp target to stay within limits
        horizontalSlideTargetPosition = Math.max(0, Math.min(1650, horizontalSlideTargetPosition));

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