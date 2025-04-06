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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Dougie Auto Align + Slide + Heading Lock")
public class DougieLimeLightVision extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    DougieArmSubSystem armSubSystem;

    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = 15;
    public static double MAX_STRAFE_POWER = 0.3;
    public static double MIN_EFFECTIVE_STRAFE_POWER = 0.2;
    public static double FINAL_ALIGNMENT_TOLERANCE = 3;

    public static double kP = 0.0065;
    public static double kI = 0.000465;
    public static double kD = 0.0002;
    public static double kF = 0.005;

    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;

    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.2;
    public static double horizontalSlideTicksPerInch = 66.5;

    private PIDController alignmentPID;
    private PIDController headingLockPID;

    private double lastCorrectedWorldY = 0;
    private double targetHeading = 0;
    private boolean alignmentFinished = false;
    private boolean slideTargetLocked = false;
    private double lockedSlideTarget = 0;

    private boolean trianglePressedLastLoop = false;

    @Override
    public void runOpMode() {

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
                armSubSystem.horizontalSlideTargetPosition = 0;
                armSubSystem.LimeLightIntakeIdlePosition();
                armSubSystem.PositionForSampleScanning();
            }
            trianglePressedLastLoop = trianglePressed;

            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            CommandScheduler.getInstance().run();

            LLResult result = limelight.getLatestResult();
            if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
                telemetry.addLine("No target detected");
                applyMotorPowers(0, getHeadingCorrection());
                telemetry.update();
                continue;
            }

            double[] output = result.getPythonOutput();
            boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;

            if (!locked) {
                telemetry.addLine("No block locked");
                applyMotorPowers(0, getHeadingCorrection());
                telemetry.update();
                continue;
            }

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

            if (!alignmentFinished && Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) {
                alignmentFinished = true;
            }

            if (alignmentFinished && !slideTargetLocked && worldY > 0) {
                double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
                correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
                lastCorrectedWorldY = correctedWorldY;

                double logCorrection = logCorrectionFactor * Math.log10(correctedWorldY);
                double scaledY = correctedWorldY + slideExtensionOffsetInches - logCorrection;
                lockedSlideTarget = scaledY * horizontalSlideTicksPerInch;
                armSubSystem.horizontalSlideTargetPosition = lockedSlideTarget;
                slideTargetLocked = true;
            }

            applyMotorPowers(alignmentFinished ? 0 : strafePower, getHeadingCorrection());

            telemetry.addData("centerX", centerX);
            telemetry.addData("targetX", targetX);
            telemetry.addData("error", error);
            telemetry.addData("StrafePower", strafePower);
            telemetry.addData("HeadingCorrection", getHeadingCorrection());
            telemetry.addData("LockedSlideTarget", lockedSlideTarget);
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

        if (Math.abs(headingError) < Math.toRadians(1.5)) {
            return 0;
        } else {
            return headingLockPID.calculate(headingError);
        }
    }
}
