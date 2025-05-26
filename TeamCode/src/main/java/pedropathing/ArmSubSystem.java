package pedropathing;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubSystem extends CommandBase {

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

    public boolean hangingCompleted = false;
    public boolean specimenCollected = false;

    public ArmSubSystem(HardwareMap hardwareMap){
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
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.6)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.6))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }


    /** Specimen Actions **/
    public void PositionForSpecimenCollection(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = -10),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.4)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.2)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.065)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(1.0)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(1.0))
                        )
                )
        );
    }

    public void PositionForSpecimenScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.7)).alongWith(
                                new InstantCommand(() -> specimenCollected = false)
                        ),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(300)),

                        new InstantCommand(() -> specimenCollected = true),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = 0),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.62)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.6)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.19)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.225)),

                                new InstantCommand(() -> verticalSlideTargetPosition = 135)
                        )
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1000).alongWith(
                                new InstantCommand(() -> hangingCompleted = false)
                        ),
                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),
                        new WaitCommand(150),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.3)).alongWith(
                                new InstantCommand(() -> hangingCompleted = true)
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
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

                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.55)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.55))
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
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.235 )),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.24))
                        )
                )
        );
    }

    public void CollectSample(){
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


    public void VerticalPIDFSlideControl() {
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