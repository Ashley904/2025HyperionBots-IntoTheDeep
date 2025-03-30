// File: DougieArmSubSystem.java
package pedropathing;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DougieArmSubSystem extends CommandBase {

    private static final double verticalSlideKp = 0.0065;
    private static final double verticalSlideKi = 0;
    private static final double verticalSlideKd = 0;
    private static final double verticalSlideKf = 0.002;
    private final double verticalSlideTicksInDegrees = 3.96;

    double verticalSlideTargetPosition;
    double currentVerticalSlidePosition;

    private static final double horizontalSlideKp = 0.0075;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.000;
    private static final double horizontalSlideKf = 0.0002;
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

        verticalLeftServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalLeftServo"), 0.75, true);
        verticalRightServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalRightServo"), 0.75, false);
        verticalControlServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalControlServo"), 1.0, false);
        verticalGripperServo = new MotionProfiledServo(hardwareMap.get(Servo.class, "verticalGripperServo"), 1.2, false);
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

    public void VerticalArmIdlePosition(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.125)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.65)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    public void PositionForSpecimenCollection(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 0),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.25)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.795)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.45)),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0))
                        )
                )
        );
    }

    public void PositionForSpecimenScoring(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalGripperServo.setTargetPosition(0.515)),
                        new WaitUntilCommand(() -> verticalGripperServo.isAtTarget(185)),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = 300),
                                new InstantCommand(() -> verticalLeftServo.setTargetPosition(0.54)),
                                new InstantCommand(() -> verticalRightServo.setTargetPosition(0.54)),
                                new InstantCommand(() -> verticalRotationServo.setTargetPosition(0.795)),
                                new InstantCommand(() -> verticalControlServo.setTargetPosition(0.65))
                        )
                )
        );
    }

    public void ScoreSpecimen(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 825),
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

    public void VerticalPIDFSlideControl() {
        verticalSlidePIDController.setPID(verticalSlideKp, verticalSlideKi, verticalSlideKd);
        currentVerticalSlidePosition = -verticalSlideLeft.getCurrentPosition();
        double pid = verticalSlidePIDController.calculate(currentVerticalSlidePosition, verticalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(verticalSlideTargetPosition / verticalSlideTicksInDegrees)) * verticalSlideKf;
        double adjustment = pid + feedForward;
        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));
        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);
        isVerticalSlideAtTarget = Math.abs(currentVerticalSlidePosition - verticalSlideTargetPosition) < 40;
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