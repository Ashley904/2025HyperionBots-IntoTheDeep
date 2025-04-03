// File: DougieArmSubSystem.java
package pedropathing;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DougieArmSubSystem extends CommandBase {

    private static final double verticalSlideKp = 0.023;
    private static final double verticalSlideKi = 0.0;
    private static final double verticalSlideKd = 0.0004;
    private static final double verticalSlideKf = 0.05;
    private final double verticalSlideTicksInDegrees = 3.96;
    double verticalSlideTargetPosition;



    private static final double horizontalSlideKp = 0.0125;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.0006;
    private static final double horizontalSlideKf = 0.25;
    private final double horizontalSlideTicksInDegrees = 3.96;

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

    private boolean shouldUpdateServos = true;


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

    public void OuttakeIdlePosition(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.3)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.2)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.65)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.25))
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
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.3)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.87)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.184)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.15)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.15))
                        )
                )
        );
    }

    public void PositionForSpecimenScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.625)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(50)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = 190),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.585)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.585)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.87)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.375))
                        )
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 790),
                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.3)),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void PositionForSampleScanning(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1050),

                        new InstantCommand(() -> LimeLightIntakeIdlePosition())
                )
        );
    }


    /** Sample Actions **/

    void IntakeIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = 235),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.5)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.6)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.25))
                        ),

                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(100)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.39)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.39))
                        ),

                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget()),
                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget()),

                        new InstantCommand(() -> horizontalSlideTargetPosition = 0)
                )
        );
    }

    public void PositionForSampleCollection(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = 750),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.165)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.165)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.435)),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.5)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.22))
                        )
                )
        );
    }

    public void LimelightPositionForSampleCollection() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.5)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(20)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.1215)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.1215)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.3815))
                        )
                )
        );
    }


    public void LimelightCollectSample() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.1)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.1)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.4))
                        ),

                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget(25)),
                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget(25)),
                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(25)),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(100)),


                        new InstantCommand(this::LimeLightIntakeIdlePosition)
                )
        );
    }


    public void LimeLightIntakeIdlePosition() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.25))
                        ),
                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(100)),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.485)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.485))
                        ),
                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget()),
                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget()),

                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget()),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.025)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget()),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0))


                )
        );
    }



    public void CollectSample(){
        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.11)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.11)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.435))
                        ),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.0125)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(215)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.38)),
                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.38)),
                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.435)),
                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.25)),

                                new WaitCommand(100),
                                new InstantCommand(() -> horizontalSlideTargetPosition = -25)
                        ),
                        new WaitCommand(650),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget()),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.065)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(150)),

                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0))


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

                        new WaitCommand(250),
                        new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.5)),
                        new WaitUntilCommand(() -> horizontalGripperServo.isAtTarget(50)),

                        new InstantCommand(this::IntakeIdlePosition)
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
                                                new InstantCommand(() -> horizontalSlideTargetPosition = 1005),
                                                new InstantCommand(() -> horizontalRotationServo.setTargetPosition(0.255)),
                                                new InstantCommand(() -> horizontalLeftServo.setTargetPosition(0.675)),
                                                new InstantCommand(() -> horizontalRightServo.setTargetPosition(0.675)),
                                                new InstantCommand(() -> horizontalControlServo.setTargetPosition(0.515)),
                                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0))
                                        ),
                                        new WaitUntilCommand(() -> horizontalLeftServo.isAtTarget()),
                                        new WaitUntilCommand(() -> horizontalRightServo.isAtTarget())
                                ),

                                // Outtake Actions
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> verticalSlideTargetPosition = 490),
                                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.22)),
                                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.325))
                                        ),
                                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),
                                        new WaitUntilCommand(() -> verticalRotationServo.isAtTarget()),
                                        new ParallelCommandGroup(
                                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(1)),
                                                new InstantCommand(() -> verticalRightServo.setTargetPosition(1))
                                        ),
                                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(50)),
                                        new WaitUntilCommand(() -> verticalRightServo.isAtTarget(50)),
                                        new WaitUntilCommand(() -> horizontalControlServo.isAtTarget(150))
                                )
                        ),

                        new SequentialCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.65)),
                                new WaitUntilCommand(() -> verticalGripperServo.isAtTarget()),
                                new InstantCommand(() -> horizontalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(this::IntakeIdlePosition),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.5)),
                                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.5))
                                )
                        ),

                        new InstantCommand(this::PositionForHighBucketScoring)
                )
        );
    }



    void PositionForHighBucketScoring(){

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 2300),
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0)),

                        new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.4)),
                        new InstantCommand(() -> verticalRightServo.setTargetPosition(0.4)),

                        new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.565))
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
                        new InstantCommand(() -> OuttakeIdlePosition())
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

        isVerticalSlideAtTarget = Math.abs(currentVerticalSlidePosition - verticalSlideTargetPosition) < 50;
    }

    public void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);
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