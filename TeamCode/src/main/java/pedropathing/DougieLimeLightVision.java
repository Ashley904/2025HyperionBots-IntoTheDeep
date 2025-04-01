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
    public static double FINAL_ALIGNMENT_TOLERANCE = 3.5;
    public static double MAX_STRAFE_POWER = 0.3;

    public static double kP = 0.01, kI = 0.002, kD = 0.5;

    private PIDController mainPIDController;
    private PIDController horizontalSlidePIDController;

    private static final double horizontalSlideKp = 0.006;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.00005;
    private static final double horizontalSlideKf = 0.00015;

    public static double horizontalSlideTicksPerInch = 65;
    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.5;

    double horizontalSlideTargetPosition;
    double currentHorizontalSlidePosition;

    private boolean positionAligned = false;
    private boolean slideReady = false;
    private boolean visionEnabled = false;

    private double savedWorldY = 0;
    private double savedAngle = 0;
    private long alignmentCompleteTime = 0;
    private double lastCorrectedWorldY = 0;
    private int alignmentHoldCounter = 0;

    private boolean sampleFrozen = false;
    private double frozenWorldY = 0;
    private double frozenAngle = 0;

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
            double strafePower = 0;

            if (gamepad1.y) {
                armSubSystem.PositionForSampleScanning();

                visionEnabled = true;
                positionAligned = false;
                slideReady = false;
                alignmentHoldCounter = 0;
                sampleFrozen = false;
                telemetry.addLine("[INFO] Vision tracking enabled!");
            }

            if (!visionEnabled) {
                applyMotorPowers(0, 0, 0);
                telemetry.addLine("[INFO] Waiting for triangle button press to start vision alignment.");
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
                telemetry.addLine("[ERROR] No Limelight result or incomplete SnapScript output");
                applyMotorPowers(0, 0, 0);
                telemetry.update();
                continue;
            }

            double[] output = result.getPythonOutput();

            boolean lockedYellow = output[0] == 1.0;
            boolean lockedRed = output[1] == 1.0;
            boolean lockedBlue = output[2] == 1.0;
            boolean targetLocked = lockedYellow || lockedRed || lockedBlue;

            double angle = output[3];
            double centerX = output[4];
            double worldY = output[6];

            double error = centerX - IMAGE_CENTER_X;

            double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
            correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
            lastCorrectedWorldY = correctedWorldY;

            if (targetLocked && !positionAligned) {
                mainPIDController.setPID(kP, kI, kD);
                strafePower = -mainPIDController.calculate(centerX, IMAGE_CENTER_X);

// Clamp strafePower to avoid extreme values
                strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));


                if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) {
                    alignmentHoldCounter++;
                } else {
                    alignmentHoldCounter = 0;
                }

                if (!sampleFrozen && alignmentHoldCounter >= 2) {
                    frozenWorldY = correctedWorldY;
                    frozenAngle = angle;
                    sampleFrozen = true;
                }

                if (alignmentHoldCounter >= 3 && Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE * 1.5) {
                    positionAligned = true;
                    alignmentCompleteTime = System.currentTimeMillis();
                    savedWorldY = correctedWorldY;
                    savedAngle = angle;
                }
            } else {
                strafePower = 0;
            }

            if (positionAligned && !slideReady && System.currentTimeMillis() - alignmentCompleteTime >= 250) {
                slideReady = true;
                if (!sampleFrozen) {
                    frozenWorldY = correctedWorldY;
                    frozenAngle = angle;
                    sampleFrozen = true;
                }
            }

            double slideYToUse = sampleFrozen ? frozenWorldY : savedWorldY;
            if ((slideReady || sampleFrozen) && slideYToUse > 0) {
                double logCorrection = logCorrectionFactor * Math.log10(slideYToUse);
                double scaledY = slideYToUse + slideExtensionOffsetInches - logCorrection;
                horizontalSlideTargetPosition = scaledY * horizontalSlideTicksPerInch;
            } else {
                horizontalSlideTargetPosition = 0;
            }

            double clampedAngle = Math.max(-150.0, Math.min(150.0, frozenAngle));
            double servoPosition = (clampedAngle + 150.0) / 300.0;
            gripperRotServo.setPosition(servoPosition);

            HorizontalPIDFSlideControl();
            armSubSystem.VerticalPIDFSlideControl();
            applyMotorPowers(0, strafePower, 0);
            telemetry.update();
        }
    }

    private void applyMotorPowers(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }

    private void HorizontalPIDFSlideControl() {
        horizontalSlidePIDController.setPID(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);
        currentHorizontalSlidePosition = horizontalSlide.getCurrentPosition();

        double pid = horizontalSlidePIDController.calculate(currentHorizontalSlidePosition, horizontalSlideTargetPosition);
        double feedForward = Math.cos(Math.toRadians(horizontalSlideTargetPosition / horizontalSlideTicksPerInch)) * horizontalSlideKf;
        double adjustment = pid + feedForward;

        adjustment = Math.max(-1.0, Math.min(1.0, adjustment));
        horizontalSlide.setPower(adjustment);
    }
}
