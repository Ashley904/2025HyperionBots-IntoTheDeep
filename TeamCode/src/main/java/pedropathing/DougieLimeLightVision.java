package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "AutoAlign_PID_DirectDrive")
public class DougieLimeLightVision extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public static double IMAGE_CENTER_X = 320.0;
    public static double FINAL_ALIGNMENT_TOLERANCE = 25.0;
    public static double MAX_STRAFE_POWER = 0.45;
    public static double FINE_TUNE_THRESHOLD = 45.0;

    public static double kP = 0.0035, kI = 0.005, kD = 4.75;
    public static double kP_fine = 145, kI_fine = 0.0, kD_fine = 0.0;

    private PIDController mainPIDController;
    private PIDController finePIDController;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        mainPIDController = new PIDController(kP, kI, kD);
        finePIDController = new PIDController(kP_fine, kI_fine, kD_fine);

        telemetry.addLine("[DEBUG] Ready to align using Dual PID (FTCLib)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double strafePower = 0;

            LLResult result = limelight.getLatestResult();
            if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 6) {
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

            double centerX = output[4];
            double error = centerX - IMAGE_CENTER_X;

            telemetry.addData("Target Locked", targetLocked);
            telemetry.addData("Center X", centerX);
            telemetry.addData("Error (pixels)", error);

            if (targetLocked && Math.abs(error) > FINAL_ALIGNMENT_TOLERANCE) {
                boolean useFinePID = Math.abs(error) < FINE_TUNE_THRESHOLD;
                PIDController activePID = useFinePID ? finePIDController : mainPIDController;

                activePID.setPID(useFinePID ? kP_fine : kP, useFinePID ? kI_fine : kI, useFinePID ? kD_fine : kD);
                strafePower = -activePID.calculate(centerX, IMAGE_CENTER_X);

                double errorRatio = Math.min(1.0, Math.abs(error) / 80.0);
                double maxPowerAllowed = errorRatio * MAX_STRAFE_POWER;
                strafePower = Math.max(-maxPowerAllowed, Math.min(maxPowerAllowed, strafePower));

                telemetry.addLine("[DEBUG] Using " + (useFinePID ? "Fine PID (FTCLib)" : "Main PID (FTCLib)"));
            } else {
                strafePower = 0;
                telemetry.addLine("[DEBUG] Target aligned or not locked");
            }

            applyMotorPowers(0, strafePower, 0);
            telemetry.addData("Strafe Power", strafePower);
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
}
