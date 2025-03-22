package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DougieArmSubSystem extends SubsystemBase {

    Servo gripperServo;
    Servo controlServo;
    Servo horizontalLeftServo;
    Servo horizontalRightServo;
    Servo rotationServo;

    DcMotor verticalSlideLeft;
    DcMotor verticalSlideRight;
    DcMotor horizontalSlide;

    private static final double kP = 0.006, kI = 0.0, kD = 0.0;
    private static final double kF = 0.002;
    static double horizontalKp = 0.006, horizontalKi = 0.0, horizontalKd = 0.0;
    static double horizontalKf = 0.0;

    final double ticksInDegrees = 2.09;
    final double horizontalTicksInDegrees = 2.09;

    int verticalSlideTargetPosition = 0;
    int horizontalSlideTargetPosition = 0;

    int currentSlidePosition = 0;
    int currentHorizontalSlidePosition = 0;

    PIDController pidController;
    PIDController horizontalPidController;


    public boolean isManualOverrideEnabled = false;
    private double rotationServoDefaultPosition = 0.165;
    public boolean isInSampleCollectionMode = false;

    public DougieArmSubSystem(HardwareMap hardwareMap) {
        pidController = new PIDController(kP, kI, kD);
        horizontalPidController = new PIDController(horizontalKp, horizontalKi, horizontalKd);

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

        horizontalLeftServo = hardwareMap.get(Servo.class, "horizontalLeftServo");
        horizontalLeftServo.setDirection(Servo.Direction.FORWARD);

        horizontalRightServo = hardwareMap.get(Servo.class, "horizontalRightServo");
        horizontalRightServo.setDirection(Servo.Direction.REVERSE);

        controlServo = hardwareMap.get(Servo.class, "controlServo");
        controlServo.setDirection(Servo.Direction.FORWARD);

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperServo.setDirection(Servo.Direction.FORWARD);

        rotationServo = hardwareMap.get(Servo.class, "gripperRotation");
        rotationServo.setDirection(Servo.Direction.FORWARD);
    }

    void PositionForSpecimenCollection() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = 0),
                        new WaitCommand(150),
                        new InstantCommand(() -> controlServo.setPosition(0.2)),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.135)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.135)),
                                new InstantCommand(() -> rotationServo.setPosition(0.115)),
                                new InstantCommand(() -> gripperServo.setPosition(0.225))
                        )
                )
        );
    }

    void PositionForSpecimenHanging() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> gripperServo.setPosition(0.52)),

                        new WaitCommand(365),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.6915)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.6915)),

                                new WaitCommand(150),
                                new InstantCommand(() -> controlServo.setPosition(0.515)),
                                new WaitCommand(300),
                                new InstantCommand(() -> rotationServo.setPosition(0.8))

                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> horizontalSlideTargetPosition = 300)
                )
        );
    }

    void ScoreSpecimenOntoHighBar() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(

                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> horizontalLeftServo.setPosition(0.9)),
                                        new InstantCommand(() -> horizontalRightServo.setPosition(0.9)),
                                        new InstantCommand(() -> gripperServo.setPosition(0.66)),
                                        new InstantCommand(() -> controlServo.setPosition(0.4)),
                                        new InstantCommand(() -> horizontalSlideTargetPosition = 1350)
                                ),

                                new WaitCommand(650),

                                new ParallelCommandGroup(
                                        new InstantCommand(() -> horizontalLeftServo.setPosition(0.7)),
                                        new InstantCommand(() -> horizontalRightServo.setPosition(0.7)),
                                        new InstantCommand(() -> gripperServo.setPosition(0.225)),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> horizontalSlideTargetPosition = 0),
                                        new InstantCommand(() -> controlServo.setPosition(0))

                                )
                        )

                )
        );
    }

    void PositionForSampleCollection(double rotationServoInput) {
        isInSampleCollectionMode = true;

        rotationServoDefaultPosition += rotationServoInput * 0.024;
        rotationServoDefaultPosition = Math.max(0.0, Math.min(1.0, rotationServoDefaultPosition));

        rotationServo.setPosition(rotationServoDefaultPosition);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.9215)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.9215)),
                                new InstantCommand(() -> controlServo.setPosition(0.1)),
                                new InstantCommand(() -> gripperServo.setPosition(0.25))
                        )
                )
        );
    }

    void CollectSample(){
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> controlServo.setPosition(0.12)),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.96)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.96))
                        ),
                        new InstantCommand(() -> gripperServo.setPosition(0.6))
                )
        );
    }

    void IdlePosition() {
        isInSampleCollectionMode = false;
        horizontalSlideTargetPosition = 0;

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlideTargetPosition = 0),
                        new InstantCommand(() -> gripperServo.setPosition(0.75)),
                        new InstantCommand(() -> rotationServo.setPosition(0.12)),
                        new InstantCommand(() -> controlServo.setPosition(1)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> horizontalLeftServo.setPosition(0.965)),
                        new InstantCommand(() -> horizontalRightServo.setPosition(0.965))
                )
        );
    }



    void VerticalPIDFSlideControl() {
        pidController.setPID(kP, kI, kD);

        currentSlidePosition = (-verticalSlideLeft.getCurrentPosition() + -verticalSlideRight.getCurrentPosition()) / 2;

        // Calculate PID and feedforward
        double pid = pidController.calculate(currentSlidePosition, verticalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(verticalSlideTargetPosition / ticksInDegrees)) * kF;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);
    }

    void HorizontalPIDFControl() {
        if (isManualOverrideEnabled) return;

        horizontalPidController.setPID(horizontalKp, horizontalKi, horizontalKd);

        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = horizontalPidController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalTicksInDegrees)) * horizontalKf;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        horizontalSlide.setPower(adjustment);
    }

    void ManualHorizontalSlideControl(double horizontalSlideInput) {
        if (isInSampleCollectionMode) {
            double scaledInput = bicubicScaling(horizontalSlideInput);
            horizontalSlideTargetPosition += scaledInput * 65;

            horizontalSlideTargetPosition = Math.max(0, Math.min(3900, horizontalSlideTargetPosition));

            UpdateHorizontalSlidePIDFControl();
        }
    }

    // Bicubic scaling function for fine control
    private double bicubicScaling(double input) {
        double a = 0.45;
        double b = 0.4;

        return a * Math.pow(input, 3) + b * input;
    }


    // Update horizontal and vertical PIDF Loops
    void UpdateVerticalSlidePIDFControl() {
        VerticalPIDFSlideControl();
    }

    void UpdateHorizontalSlidePIDFControl() {
        HorizontalPIDFControl();
    }

}