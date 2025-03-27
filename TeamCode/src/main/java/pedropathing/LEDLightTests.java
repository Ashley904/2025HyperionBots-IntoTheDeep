package pedropathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LED Light test")
public class LEDLightTests extends LinearOpMode {

    Servo ledServo;
    DigitalChannel led;

    public void runOpMode(){

        ledServo = hardwareMap.get(Servo.class, "LEDServo");

        telemetry.addData("Status:", "Ready to start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(isStopRequested()) return;

            ledServo.setPosition(1.0);
        }

    }



}
