package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "AutoAlign_PID_DirectDrive")
public class DougieLimeLightVision extends LinearOpMode {

    DougieArmSubSystem armSubSystem;

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor horizontalSlide;
    private Servo gripperRotServo;

    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -14.85;
    public static double FINAL_ALIGNMENT_TOLERANCE = 3;
    public static double MAX_STRAFE_POWER = 0.3;

    public static double kP = 0.0125, kI = 0.05, kD = 0.0;

    private PIDController mainPIDController;
    private PIDController horizontalSlidePIDController;

    private static final double horizontalSlideKp = 0.006;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.00005;
    private static final double horizontalSlideKf = 0.00015;

    public static double horizontalSlideTicksPerInch = 65;
    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.2;

    double horizontalSlideTargetPosition;
    double currentHorizontalSlidePosition;

    private double savedWorldY = 0;
    private double savedAngle = 0;
    private double savedServoPosition = 0.5;
    private long alignmentCompleteTime = 0;
    private double lastCorrectedWorldY = 0;
    private int alignmentHoldCounter = 0;

    private boolean sampleFrozen = false;
    private double frozenWorldY = 0;
    private double frozenAngle = 0;

    private boolean trianglePressedLastLoop = false;
    private boolean hasStartedSlide = false;
    private boolean hasSetServoAngle = false;

    private enum VisionState {
        IDLE,
        ALIGNING,
        ROTATING_SERVO,
        SLIDE_EXTENDING,
        COLLECTING
    }

    private VisionState visionState = VisionState.IDLE;

    @Override
    public void runOpMode() {

        armSubSystem = new DougieArmSubSystem(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        horizontalSlide = hardwareMap.get(DcMotor.class, "HorizontalSlide");
        horizontalSlide.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripperRotServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        mainPIDController = new PIDController(kP, kI, kD);
        horizontalSlidePIDController = new PIDController(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean trianglePressed = gamepad1.y;

            if (trianglePressed && !trianglePressedLastLoop) {
                armSubSystem.LimeLightIntakeIdlePosition(); // Reset arm to idle
                armSubSystem.PositionForSampleScanning();   // Go to scanning pose
                horizontalSlideTargetPosition = 0;          // Reset slide to 0
                visionState = VisionState.ALIGNING;
                alignmentHoldCounter = 0;
                sampleFrozen = false;
                hasStartedSlide = false;
                hasSetServoAngle = false;
                telemetry.addLine("[INFO] New sample requested. Scanning...");
            }

            trianglePressedLastLoop = trianglePressed;

            if (visionState == VisionState.IDLE) {
                applyMotorPowers(0, 0, 0);
                telemetry.addLine("[INFO] Waiting for triangle press.");
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
                telemetry.addLine("[ERROR] Limelight missing or invalid output");
                applyMotorPowers(0, 0, 0);
                telemetry.update();
                continue;
            }

            double[] output = result.getPythonOutput();

            boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;
            double angle = output[3];
            double centerX = output[4];
            double worldY = output[6];
            double adjustedCenterX = IMAGE_CENTER_X + ALIGNMENT_OFFSET;
            double error = centerX - adjustedCenterX;

            double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
            correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
            lastCorrectedWorldY = correctedWorldY;

            double strafePower = 0;

            if (visionState == VisionState.ALIGNING && locked) {
                strafePower = -mainPIDController.calculate(centerX, adjustedCenterX);
                strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));

                if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) alignmentHoldCounter++;
                else alignmentHoldCounter = 0;

                if (!sampleFrozen && alignmentHoldCounter >= 2) {
                    frozenWorldY = correctedWorldY;
                    frozenAngle = angle;
                    sampleFrozen = true;
                }

                if (alignmentHoldCounter >= 3 && Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE * 1.5) {
                    visionState = VisionState.ROTATING_SERVO;
                    savedWorldY = correctedWorldY;
                    savedAngle = angle;
                    savedServoPosition = (Math.max(-150.0, Math.min(150.0, savedAngle)) + 150.0) / 300.0;
                    alignmentCompleteTime = System.currentTimeMillis();
                }
            }

            if (visionState == VisionState.ROTATING_SERVO) {
                gripperRotServo.setPosition(savedServoPosition);
                if (System.currentTimeMillis() - alignmentCompleteTime >= 200) {
                    visionState = VisionState.SLIDE_EXTENDING;
                }
            }

            if (visionState == VisionState.SLIDE_EXTENDING) {
                double slideY = sampleFrozen ? frozenWorldY : savedWorldY;
                if (slideY > 0) {
                    double logCorrection = logCorrectionFactor * Math.log10(slideY);
                    double scaledY = slideY + slideExtensionOffsetInches - logCorrection;
                    horizontalSlideTargetPosition = scaledY * horizontalSlideTicksPerInch;
                }

                if (!hasStartedSlide) {
                    armSubSystem.LimelightPositionForSampleCollection();
                    hasStartedSlide = true;
                }

                if (Math.abs(currentHorizontalSlidePosition - horizontalSlideTargetPosition) <= 15) {
                    visionState = VisionState.COLLECTING;
                    armSubSystem.LimelightCollectSample();
                }
            }

            HorizontalPIDFSlideControl();
            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.updateServos();
            CommandScheduler.getInstance().run();
            applyMotorPowers(0, strafePower, 0);

            telemetry.addData("State", visionState);
            telemetry.update();
        }
    }

    private void applyMotorPowers(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);
        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();

        double pid = horizontalSlidePIDController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double ff = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalSlideTicksPerInch)) * horizontalSlideKf;
        double output = Math.max(-1.0, Math.min(1.0, pid + ff));

        horizontalSlide.setPower(output);
    }
}
