package pedropathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Limelight rotation servo calibration")
public class LimelightServoCalibration extends LinearOpMode {

    DougieArmSubSystem armSubSystem;

    public void runOpMode(){

        armSubSystem = new DougieArmSubSystem(hardwareMap);

        telemetry.addData("Status: ", "Ready to start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            armSubSystem.PositionForSampleScanning();

            // Toggling the different rotation servo positions
            if(gamepad1.a) armSubSystem.horizontalRotationServo.setTargetPosition(0);
            else if(gamepad1.y) armSubSystem.horizontalRotationServo.setTargetPosition(1);

            armSubSystem.updateServos();
            armSubSystem.VerticalPIDFSlideControl();

        }
    }



}
