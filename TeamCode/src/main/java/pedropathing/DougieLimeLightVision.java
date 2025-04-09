package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Dougie Auto Align + Slide + Heading Lock")
public class DougieLimeLightVision extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private Servo rotationServo;

    DougieArmSubSystem armSubSystem;

    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -20;
    public static double MAX_STRAFE_POWER = 0.185;
    public static double MIN_EFFECTIVE_STRAFE_POWER = 0.16;
    public static double FINAL_ALIGNMENT_TOLERANCE = 3;
    public static int REQUIRED_STABLE_FRAMES = 7;

    public static double kP = 0.0065;
    public static double kI = 0.000465;
    public static double kD = 0.0002;
    public static double kF = 0.0055;

    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;

    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.05;
    public static double horizontalSlideTicksPerInch = 58.65;

    public static double ROTATION_OFFSET_DEGREES = 66;
    public static int SERVO_ROTATION_DELAY_MS = 1000; // New: Delay after servo rotation before collection

    private PIDController alignmentPID;
    private PIDController headingLockPID;

    private double targetHeading = 0;
    private double lastCorrectedWorldY = 0;
    private double smoothedAngle = 0;
    private double lockedAngle = 0;
    private double lockedSlideTarget = 0;
    private double lastRotationOffsetApplied = Double.NaN;

    private boolean alignmentFinished = false;
    private boolean slideTargetLocked = false;
    private boolean trianglePressedLastLoop = false;
    private boolean waitingForSlideRetraction = false;
    private boolean rotationServoPositionLocked = false;
    private boolean collectionTriggered = false;
    private boolean servoRotationStarted = false; // New: Track if servo rotation has started
    private long servoRotationStartTime = 0; // New: Track when servo rotation started

    private int stableFrameCount = 0;
    private double lastStableX = 0;

    @Override
    public void runOpMode() {
        rotationServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");
        rotationServo.setDirection(Servo.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        alignmentPID = new PIDController(kP, kI, kD);
        headingLockPID = new PIDController(headingKp, headingKi, headingKd);
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        telemetry.addLine("Ready to align...");
        telemetry.update();
        waitForStart();

        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            boolean trianglePressed = gamepad1.y;

            if (trianglePressed && !trianglePressedLastLoop) {
                alignmentFinished = false;
                slideTargetLocked = false;
                waitingForSlideRetraction = true;
                rotationServoPositionLocked = false;
                collectionTriggered = false;
                servoRotationStarted = false;
                stableFrameCount = 0;
                armSubSystem.horizontalSlideTargetPosition = 0;
                armSubSystem.LimeLightIntakeIdlePosition();
                armSubSystem.PositionForSampleScanning();
            }
            trianglePressedLastLoop = trianglePressed;

            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();
            CommandScheduler.getInstance().run();

            if (waitingForSlideRetraction && Math.abs(armSubSystem.currentHorizontalSlidePosition) <= 50) {
                waitingForSlideRetraction = false;
            }

            if (waitingForSlideRetraction) {
                telemetry.addLine("Waiting for slide to retract...");
                applyMotorPowers(0, getHeadingCorrection());
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
                telemetry.addLine("No target detected");
                stableFrameCount = 0;
                applyMotorPowers(0, getHeadingCorrection());
                telemetry.update();
                continue;
            }

            double[] output = result.getPythonOutput();
            boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;
            if (!locked) {
                telemetry.addLine("No block locked");
                stableFrameCount = 0;
                applyMotorPowers(0, getHeadingCorrection());
                telemetry.update();
                continue;
            }

            double angle = output[3];
            double centerX = output[4];
            double worldY = output[6];
            double targetX = IMAGE_CENTER_X + ALIGNMENT_OFFSET;
            double error = centerX - targetX;

            alignmentPID.setPID(kP, kI, kD);
            double pidOutput = -alignmentPID.calculate(centerX, targetX);
            double ff = Math.signum(error) * kF;
            double strafePower = pidOutput + ff;

            if (Math.abs(strafePower) < MIN_EFFECTIVE_STRAFE_POWER && Math.abs(error) > FINAL_ALIGNMENT_TOLERANCE) {
                strafePower = Math.copySign(MIN_EFFECTIVE_STRAFE_POWER, strafePower);
            }
            strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));

            // Check for stable alignment
            if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) {
                if (stableFrameCount == 0 || Math.abs(centerX - lastStableX) < 2.0) {
                    stableFrameCount++;
                    lastStableX = centerX;
                } else {
                    stableFrameCount = 0;
                }
            } else {
                stableFrameCount = 0;
            }

            if (!alignmentFinished && stableFrameCount >= REQUIRED_STABLE_FRAMES) {
                alignmentFinished = true;
            }

            smoothedAngle = 0.8 * smoothedAngle + 0.2 * angle;

            // Start servo rotation as soon as we have a stable angle, even before full alignment
            if (!servoRotationStarted && stableFrameCount >= 3) { // Changed: Start rotation earlier
                lockedAngle = ((smoothedAngle % 360) + 360) % 360;
                if (lockedAngle > 180) lockedAngle = 360 - lockedAngle;

                double finalAngle = lockedAngle + ROTATION_OFFSET_DEGREES;
                finalAngle = Math.max(0.0, Math.min(180.0, finalAngle));
                double mappedServoPosition = finalAngle / 180.0;
                armSubSystem.horizontalRotationServo.setTargetPosition(mappedServoPosition);

                lastRotationOffsetApplied = ROTATION_OFFSET_DEGREES;
                servoRotationStarted = true;
                servoRotationStartTime = System.currentTimeMillis();
                rotationServoPositionLocked = true;
            }

            if (rotationServoPositionLocked && ROTATION_OFFSET_DEGREES != lastRotationOffsetApplied) {
                double finalAngle = lockedAngle + ROTATION_OFFSET_DEGREES;
                finalAngle = Math.max(0.0, Math.min(180.0, finalAngle));
                double mappedServoPosition = finalAngle / 180.0;
                armSubSystem.horizontalRotationServo.setTargetPosition(mappedServoPosition);

                lastRotationOffsetApplied = ROTATION_OFFSET_DEGREES;
            }

            if (alignmentFinished && rotationServoPositionLocked && !slideTargetLocked && worldY > 0) {
                double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
                correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
                lastCorrectedWorldY = correctedWorldY;

                double logCorrection = logCorrectionFactor * Math.log10(correctedWorldY);
                double scaledY = correctedWorldY + slideExtensionOffsetInches - logCorrection;
                lockedSlideTarget = scaledY * horizontalSlideTicksPerInch;
                armSubSystem.horizontalSlideTargetPosition = lockedSlideTarget;

                slideTargetLocked = true;
            }

            // Only proceed with collection if:
            // 1. We're aligned
            // 2. Slide is at target
            // 3. Servo has had time to rotate (500ms)
            if (alignmentFinished && rotationServoPositionLocked && slideTargetLocked && !collectionTriggered) {
                long timeSinceServoRotation = System.currentTimeMillis() - servoRotationStartTime;
                if (timeSinceServoRotation >= SERVO_ROTATION_DELAY_MS) {
                    armSubSystem.LimelightPositionForSampleCollection();
                    sleep(300);
                    armSubSystem.LimelightCollectSample();
                    collectionTriggered = true;
                }
            }

            applyMotorPowers(alignmentFinished ? 0 : strafePower, getHeadingCorrection());

            telemetry.addData("centerX", centerX);
            telemetry.addData("targetX", targetX);
            telemetry.addData("error", error);
            telemetry.addData("StrafePower", strafePower);
            telemetry.addData("HeadingCorrection", getHeadingCorrection());
            telemetry.addData("LockedSlideTarget", lockedSlideTarget);
            telemetry.addData("SmoothedAngle", smoothedAngle);
            telemetry.addData("LockedAngle", lockedAngle);
            telemetry.addData("OffsetAngle", ROTATION_OFFSET_DEGREES);
            telemetry.addData("FinalAngle", lockedAngle + ROTATION_OFFSET_DEGREES);
            telemetry.addData("ServoTargetPos", (lockedAngle + ROTATION_OFFSET_DEGREES) / 180.0);
            telemetry.addData("StableFrameCount", stableFrameCount);
            telemetry.addData("CollectionTriggered", collectionTriggered);
            telemetry.addData("TimeSinceServoRotation", System.currentTimeMillis() - servoRotationStartTime);
            telemetry.update();
        }
    }

    private void applyMotorPowers(double strafe, double headingCorrection) {
        double fl = strafe + headingCorrection;
        double fr = -strafe - headingCorrection;
        double bl = -strafe + headingCorrection;
        double br = strafe - headingCorrection;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private double getHeadingCorrection() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = targetHeading - currentHeading;
        headingError = Math.IEEEremainder(headingError, 2 * Math.PI);
        if (Math.abs(headingError) < Math.toRadians(1.5)) return 0;
        return headingLockPID.calculate(headingError);
    }
}