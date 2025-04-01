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
    private static final double verticalSlideKd = 0.0005;
    private static final double verticalSlideKf = 0.05;
    private final double verticalSlideTicksInDegrees = 3.96;
    double verticalSlideTargetPosition;



    private static final double horizontalSlideKp = 0.0125;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.0006;
    private static final double horizontalSlideKf = 0.25;
    private final double horizontalSlideTicksInDegrees = 2.09;

    double horizontalSlideTargetPosition;
    double currentHorizontalSlidePosition;

    PIDController verticalSlidePIDController;
    PIDController horizontalSlidePIDController;
    boolean isVerticalSlideAtTarget = false;

    DcMotor verticalSlideLeft;
    DcMotor verticalSlideRight;
    DcMotor horizontalSlide;

    MotionProfiledServo verticalLeftServo;
    MotionProfiledServo verticalRightServo;
    MotionProfiledServo verticalControlServo;
    MotionProfiledServo verticalRotationServo;
    MotionProfiledServo verticalGripperServo;

    Servo horizontalLeftServo;
    Servo horizontalRightServo;
    Servo horizontalControlServo;
    Servo horizontalRotationServo;
    Servo horizontalGripperServo;

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

        horizontalLeftServo = hardwareMap.get(Servo.class, "horizontalLeftServo");
        horizontalLeftServo.setDirection(Servo.Direction.FORWARD);
        horizontalRightServo = hardwareMap.get(Servo.class, "horizontalRightServo");
        horizontalRightServo.setDirection(Servo.Direction.REVERSE);
        horizontalControlServo = hardwareMap.get(Servo.class, "horizontalControlServo");
        horizontalControlServo.setDirection(Servo.Direction.FORWARD);
        horizontalGripperServo = hardwareMap.get(Servo.class, "horizontalGripperServo");
        horizontalGripperServo.setDirection(Servo.Direction.FORWARD);
        horizontalRotationServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");
        horizontalRotationServo.setDirection(Servo.Direction.FORWARD);
    }

    public void OuttakeIdlePosition(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.125)),
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
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.79)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.125)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.125))
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
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.795)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.4))
                        )
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 775),
                        new WaitUntilCommand(() -> isVerticalSlideAtTarget),
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void PositionForSampleScanning(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1050)
                )
        );
    }


    /** Sample Actions **/
    public void TransferSampleToOuttake() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.125)),
                                new InstantCommand(() -> verticalSlideTargetPosition = 0)
                        ),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(50)),
                        new WaitUntilCommand(() -> verticalRotationServo.isAtTarget()),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.6)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.795)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.795))
                        ),

                        new WaitUntilCommand(() -> verticalControlServo.isAtTarget(50)),
                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(50)),
                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(50)),

                        new InstantCommand(() -> {
                            verticalLeftServo.setTargetPosition(0.795);
                            verticalRightServo.setTargetPosition(0.795);
                            verticalControlServo.setTargetPosition(0.6);
                        }),

                        new WaitUntilCommand(() -> verticalControlServo.isAtTarget()),
                        new WaitUntilCommand(() -> verticalLeftServo.isAtTarget(65)),
                        new WaitUntilCommand(() -> verticalRightServo.isAtTarget(65)),

                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.515)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(25)),

                        new InstantCommand(() -> PositionForHighBucketScoring())
                )
        );
    }

    void PositionForHighBucketScoring(){

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1925),
                        new InstantCommand(() -> verticalControlServo.setTargetPosition(0.1))
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
    }

    public void updateServos() {
        verticalGripperServo.update();
        verticalLeftServo.update();
        verticalRightServo.update();
        verticalControlServo.update();
        verticalRotationServo.update();
    }
}