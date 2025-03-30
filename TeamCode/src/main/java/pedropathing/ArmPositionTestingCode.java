package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Arm Position Testing")
public class ArmPositionTestingCode extends LinearOpMode {

    DcMotor verticalSlideLeft;
    DcMotor verticalSlideRight;
    DcMotor horizontalSlide;

    Servo verticalLeftServo;
    Servo verticalRightServo;
    Servo verticalControlServo;
    Servo verticalGripperServo;
    Servo verticalRotationServo;

    Servo horizontalLeftServo;
    Servo horizontalRightServo;
    Servo horizontalControlServo;
    Servo horizontalRotationServo;
    Servo horizontalGripperServo;

    public static double kP = 0.006, kI = 0.0, kD = 0.0;
    public static double horizontalKp = 0.006, horizontalKi = 0.0, horizontalKd = 0.0;
    public static double kF = 0.002;
    public static double horizontalKf = 0.0;
    final double ticksInDegrees = 3.96;
    final double horizontalTicksInDegrees = 2.09;

    public static int verticalSlideTargetPosition = 0;
    public static int horizontalSlideTargetPosition = 0;

    /** Outtake Positions **/
    public static double targetVerticalServoPosition = 0.0;
    public static double targetVerticalControlServoPosition = 0.0;
    public static double targetVerticalGripperServoPosition = 0.0;
    public static double targetVerticalRotationServoPosition = 0.0;

    /** Intake Positions **/
    public static double targetHorizontalServoPosition = 0.0;
    public static double targetHorizontalControlServoPosition = 0.0;
    public static double targetHorizontalGripperServoPosition = 0.0;
    public static double targetHorizontalRotationServoPosition = 0.0;

    int currentSlidePosition = 0;
    int currentHorizontalSlidePosition = 0;

    PIDController pidController;
    PIDController horizontalPidController;

    public void runOpMode(){
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
        horizontalSlide.setDirection(DcMotor.Direction.FORWARD);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /** Outtake Servos **/
        verticalLeftServo = hardwareMap.get(Servo.class, "verticalLeftServo");
        verticalLeftServo.setDirection(Servo.Direction.REVERSE);

        verticalRightServo = hardwareMap.get(Servo.class, "verticalRightServo");
        verticalRightServo.setDirection(Servo.Direction.FORWARD);

        verticalControlServo = hardwareMap.get(Servo.class, "verticalControlServo");
        verticalControlServo.setDirection(Servo.Direction.FORWARD);

        verticalGripperServo = hardwareMap.get(Servo.class, "verticalGripperServo");
        verticalGripperServo.setDirection(Servo.Direction.FORWARD);

        verticalRotationServo = hardwareMap.get(Servo.class, "verticalGripperRotation");
        verticalRotationServo.setDirection(Servo.Direction.FORWARD);

        /*
        /** Intake Servos
        horizontalLeftServo = hardwareMap.get(Servo.class, "horizontalLeftServo");
        horizontalLeftServo.setDirection(Servo.Direction.REVERSE);

        horizontalRightServo = hardwareMap.get(Servo.class, "horizontalRightServo");
        horizontalRightServo.setDirection(Servo.Direction.FORWARD);

        horizontalControlServo = hardwareMap.get(Servo.class, "horizontalControlServo");
        horizontalControlServo.setDirection(Servo.Direction.FORWARD);

        horizontalGripperServo = hardwareMap.get(Servo.class, "horizontalGripperServo");
        horizontalGripperServo.setDirection(Servo.Direction.REVERSE);

        horizontalRotationServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");
        horizontalRotationServo.setDirection(Servo.Direction.FORWARD);

         */


        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()){

            /** Outake **/
            verticalLeftServo.setPosition(targetVerticalServoPosition);
            verticalRightServo.setPosition(targetVerticalServoPosition);
            verticalControlServo.setPosition(targetVerticalControlServoPosition);
            verticalGripperServo.setPosition(targetVerticalGripperServoPosition);
            verticalRotationServo.setPosition(targetVerticalRotationServoPosition);

            /*
            /** Intake
            horizontalLeftServo.setPosition(targetHorizontalServoPosition);
            horizontalRightServo.setPosition(targetHorizontalServoPosition);
            horizontalControlServo.setPosition(targetHorizontalControlServoPosition);
            horizontalGripperServo.setPosition(targetHorizontalGripperServoPosition);
            horizontalRotationServo.setPosition(targetHorizontalRotationServoPosition);

             */


            HorizontalPIDFControl();
            VerticalPIDFSlideControl();
        }

    }

    void VerticalPIDFSlideControl() {
        pidController.setPID(kP, kI, kD);

        currentSlidePosition = -verticalSlideLeft.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = pidController.calculate(currentSlidePosition, verticalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(verticalSlideTargetPosition / ticksInDegrees)) * kF;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);
    }

    void HorizontalPIDFControl() {
        pidController.setPID(horizontalKp, horizontalKi, horizontalKd);

        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = horizontalPidController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalTicksInDegrees)) * horizontalKf;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));

        horizontalSlide.setPower(adjustment);
    }

}
