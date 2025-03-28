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

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor horizontalSlide;
    private DcMotor verticalSlideLeft;
    private DcMotor verticalSlideRight;
    private Servo rotationServo;

    public static double IMAGE_CENTER_X = 320.0;
    public static double FINAL_ALIGNMENT_TOLERANCE = 2.5;
    public static double MAX_STRAFE_POWER = 0.47;
    public static double FINE_TUNE_THRESHOLD = 65.0;

    public static double kP = 0.0045, kI = 0.005, kD = 3.5;
    public static double kP_fine = 200, kI_fine = 0.0001, kD_fine = 0.1;

    private PIDController mainPIDController;
    private PIDController finePIDController;
    private PIDController horizontalSlidePIDController;

    private static final double horizontalSlideKp = 0.006;
    private static final double horizontalSlideKi = 0;
    private static final double horizontalSlideKd = 0.00005;
    private static final double horizontalSlideKf = 0.00015;

    public static double horizontalSlideTicksPerInch = 40;

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

    DougieArmSubSystem armSubSystem;

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

        verticalSlideLeft = hardwareMap.get(DcMotor.class, "VerticalSlideLeft");
        verticalSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        verticalSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalSlideRight = hardwareMap.get(DcMotor.class, "VerticalSlideRight");
        verticalSlideRight.setDirection(DcMotor.Direction.FORWARD);
        verticalSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationServo = hardwareMap.get(Servo.class, "gripperRotation");
        rotationServo.setDirection(Servo.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        mainPIDController = new PIDController(kP, kI, kD);
        finePIDController = new PIDController(kP_fine, kI_fine, kD_fine);
        horizontalSlidePIDController = new PIDController(horizontalSlideKp, horizontalSlideKi, horizontalSlideKd);

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            armSubSystem.PositionForSampleScanning();
            double strafePower = 0;

            if (gamepad1.y) {
                visionEnabled = true;
                positionAligned = false;
                slideReady = false;
                alignmentHoldCounter = 0;
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

            telemetry.addData("Target Locked", targetLocked);
            telemetry.addData("Center X", centerX);
            telemetry.addData("Error (pixels)", error);
            telemetry.addData("Raw World Y", worldY);

            double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
            correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
            lastCorrectedWorldY = correctedWorldY;

            telemetry.addData("Corrected World Y", correctedWorldY);

            if (targetLocked && !positionAligned) {
                boolean useFinePID = Math.abs(error) < FINE_TUNE_THRESHOLD;
                PIDController activePID = useFinePID ? finePIDController : mainPIDController;

                activePID.setPID(useFinePID ? kP_fine : kP, useFinePID ? kI_fine : kI, useFinePID ? kD_fine : kD);
                strafePower = -activePID.calculate(centerX, IMAGE_CENTER_X);

                double errorRatio = Math.min(1.0, Math.abs(error) / 80.0);
                double maxPowerAllowed = errorRatio * MAX_STRAFE_POWER;
                strafePower = Math.max(-maxPowerAllowed, Math.min(maxPowerAllowed, strafePower));

                telemetry.addLine("[DEBUG] Aligning... Using " + (useFinePID ? "Fine PID (FTCLib)" : "Main PID (FTCLib)"));

                if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) {
                    alignmentHoldCounter++;
                } else {
                    alignmentHoldCounter = 0;
                }

                if (alignmentHoldCounter >= 3) {
                    if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE * 1.5) {
                        positionAligned = true;
                        alignmentCompleteTime = System.currentTimeMillis();
                        savedWorldY = correctedWorldY;
                        savedAngle = angle;
                        telemetry.addLine("[DEBUG] Alignment complete. Holding target consistently.");
                    } else {
                        telemetry.addLine("[DEBUG] Close to alignment but still off, applying small correction.");
                        strafePower = -0.25 * Math.signum(error);
                    }
                }
            } else {
                strafePower = 0;
                telemetry.addLine("[DEBUG] Not aligning or alignment complete.");
            }

            if (positionAligned && !slideReady && System.currentTimeMillis() - alignmentCompleteTime >= 250) {
                slideReady = true;
                telemetry.addLine("[DEBUG] Slide is READY to extend to target!");
            }

            if (slideReady && savedWorldY > 0) {
                horizontalSlideTargetPosition = savedWorldY * horizontalSlideTicksPerInch * 1.25; // Adjust as needed
            } else {
                horizontalSlideTargetPosition = 0;
            }

            HorizontalPIDFSlideControl();
            armSubSystem.VerticalPIDFSlideControl();

            CommandScheduler.getInstance().run();

            applyMotorPowers(0, strafePower, 0);

            double clampedAngle = Math.max(-150.0, Math.min(150.0, savedAngle));
            double servoPosition = (clampedAngle + 150.0) / 300.0;
            rotationServo.setPosition(servoPosition);

            telemetry.addData("Gripper Rotation (deg)", savedAngle);
            telemetry.addData("Gripper Servo Pos", servoPosition);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Slide Target Pos", horizontalSlideTargetPosition);
            telemetry.addData("Slide Current Pos", currentHorizontalSlidePosition);
            telemetry.addData("Slide PID Error", horizontalSlideTargetPosition - currentHorizontalSlidePosition);
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
