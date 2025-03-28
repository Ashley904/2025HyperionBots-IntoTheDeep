package pedropathing;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DougieArmSubSystem extends CommandBase {

    /** Vertical slide PID Variables **/
    private static final double verticalSlideKp = 0.006;
    private static final double verticalSlideKi = 0;
    private static final double verticalSlideKd = 0;
    private static final double verticalSlideKf = 0.002;

    private final double verticalSlideTicksInDegrees = 3.96;

    double verticalSlideTargetPosition;
    double currentVerticalSlidePosition;




    /** Horizontal slide PID Variables **/
    private static final double horizontalSlideKp = 0.0075;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.000;
    private static final double horizontalSlideKf = 0.0002;

    private final double horizontalSlideTicksInDegrees = 2.09;

    double horizontalSlideTargetPosition;
    double currentHorizontalSlidePosition;


    PIDController verticalSlidePIDController;
    PIDController horizontalSlidePIDController;


    DcMotor verticalSlideLeft;
    DcMotor verticalSlideRight;
    DcMotor horizontalSlide;

    Servo horizontalLeftServo;
    Servo horizontalRightServo;
    Servo controlServo;
    Servo rotationServo;
    Servo gripperServo;


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

        horizontalLeftServo = hardwareMap.get(Servo.class, "horizontalLeftServo");
        horizontalLeftServo.setDirection(Servo.Direction.REVERSE);

        horizontalRightServo = hardwareMap.get(Servo.class, "horizontalRightServo");
        horizontalRightServo.setDirection(Servo.Direction.FORWARD);

        controlServo = hardwareMap.get(Servo.class, "controlServo");
        controlServo.setDirection(Servo.Direction.FORWARD);

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperServo.setDirection(Servo.Direction.FORWARD);

        rotationServo = hardwareMap.get(Servo.class, "gripperRotation");
        rotationServo.setDirection(Servo.Direction.FORWARD);

    }

    void VerticalArmIdlePosition(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new ParallelCommandGroup(
                                new InstantCommand(() -> gripperServo.setPosition(0.25)),
                                new InstantCommand(() -> rotationServo.setPosition(0.125)),
                                new InstantCommand(() -> controlServo.setPosition(0.65)),
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0))
                        ),

                        new InstantCommand(() -> verticalSlideTargetPosition = 0)

                )
        );
    }

    void PositionForSpecimenCollection(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> verticalSlideTargetPosition = 0),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> gripperServo.setPosition(0.25)),
                                new InstantCommand(() -> rotationServo.setPosition(0.125)),
                                new InstantCommand(() -> controlServo.setPosition(0.475)),

                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.1)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.1))
                        )

                )
        );
    }

    void PositionForSpecimenScoring(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> gripperServo.setPosition(0.515)),
                        new WaitCommand(350),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> verticalSlideTargetPosition = 150),

                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.54)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.54)),

                                new InstantCommand(() -> controlServo.setPosition(0.65)),
                                new InstantCommand(() -> rotationServo.setPosition(0.125))
                        )

                )
        );
    }

    void ScoreSpecimen(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 750),

                        new WaitCommand(450),

                        new InstantCommand(() -> gripperServo.setPosition(0.25)),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    void PositionForSampleHighBasketScoring(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> gripperServo.setPosition(0.5)),
                        new WaitCommand(350),

                        new InstantCommand(() -> verticalSlideTargetPosition = 2000),

                        new WaitCommand(250),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0.7)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0.7)),

                                new InstantCommand(() -> controlServo.setPosition(0.4)),
                                new InstantCommand(() -> rotationServo.setPosition(0.125))
                        )
                )
        );
    }

    void DropSampleIntoHighBucket(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> gripperServo.setPosition(0.25)),

                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> horizontalLeftServo.setPosition(0)),
                                new InstantCommand(() -> horizontalRightServo.setPosition(0))
                        ),
                        new InstantCommand(() -> verticalSlideTargetPosition = 0)
                )
        );
    }

    void PositionForSampleScanning(){

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> verticalSlideTargetPosition = 1050)
                )
        );
    }


    void VerticalPIDFSlideControl() {
        verticalSlidePIDController.setPID(verticalSlideKp, verticalSlideKi, verticalSlideKd);

        currentVerticalSlidePosition = -verticalSlideLeft.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = verticalSlidePIDController.calculate(currentVerticalSlidePosition, verticalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(verticalSlideTargetPosition / verticalSlideTicksInDegrees)) * verticalSlideKf;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);
    }

    void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);
        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = horizontalSlidePIDController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalSlideTicksInDegrees)) * horizontalSlideKf;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        horizontalSlide.setPower(adjustment);
    }

}


