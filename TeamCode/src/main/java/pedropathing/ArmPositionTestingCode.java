package pedropathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    public static double kP = 0.023, kI = 0.0, kD = 0.0005;
    public static double kF = 0.05;
    final double ticksInDegrees = 3.96;

    public static double horizontalKp = 0.0125, horizontalKi = 0.0, horizontalKd = 0.0006;
    public static double horizontalKf = 0.25;
    final double horizontalTicksInDegrees = 3.96;

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
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
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


        /** Intake Servos **/
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


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        while(opModeIsActive()){

            /** Outake **/
            verticalLeftServo.setPosition(targetVerticalServoPosition);
            verticalRightServo.setPosition(targetVerticalServoPosition);
            verticalControlServo.setPosition(targetVerticalControlServoPosition);
            verticalGripperServo.setPosition(targetVerticalGripperServoPosition);
            verticalRotationServo.setPosition(targetVerticalRotationServoPosition);


            /** Intake **/
            horizontalLeftServo.setPosition(targetHorizontalServoPosition);
            horizontalRightServo.setPosition(targetHorizontalServoPosition);
            horizontalControlServo.setPosition(targetHorizontalControlServoPosition);
            horizontalGripperServo.setPosition(targetHorizontalGripperServoPosition);
            horizontalRotationServo.setPosition(targetHorizontalRotationServoPosition);


            HorizontalPIDFControl();
            VerticalPIDFSlideControl();
        }

    }

    void VerticalPIDFSlideControl() {
        pidController.setPID(kP, kI, kD);

        int currentVerticalSlidePosition = verticalSlideRight.getCurrentPosition();
        double PID = pidController.calculate(currentVerticalSlidePosition, verticalSlideTargetPosition);

        double feedforward = Math.cos(Math.toRadians(verticalSlideTargetPosition / ticksInDegrees)) * kF;
        double adjustment = PID + feedforward;

        verticalSlideLeft.setPower(adjustment);
        verticalSlideRight.setPower(adjustment);

        telemetry.addData("Vertical Slide Current Position: ", verticalSlideRight.getCurrentPosition());
        telemetry.addData("Vertical Slide Target Position: ", verticalSlideTargetPosition);
        telemetry.update();
    }

    void HorizontalPIDFControl() {
        horizontalPidController.setPID(horizontalKp, horizontalKi, horizontalKd);

        int currentVerticalSlidePosition = horizontalSlide.getCurrentPosition();
        double PID = horizontalPidController.calculate(currentVerticalSlidePosition, horizontalSlideTargetPosition);

        double feedforward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalTicksInDegrees)) * horizontalKf;
        double adjustment = PID + feedforward;

        horizontalSlide.setPower(adjustment);

        telemetry.addData("Horizontal Slide Current Position: ", horizontalSlide.getCurrentPosition());
        telemetry.addData("Horizontal Sldie Target Position: ", horizontalSlideTargetPosition);
        telemetry.update();
    }

}
