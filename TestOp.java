package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by voelk_000 on 11/2/2016.
 * Edited by Banks T on 11/3/2016.
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: TestOp", group = "Oscar")
@Disabled
public class TestOp extends BaseOp {
    @Override
    public void init() {
        super.init();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); // change to position run encoder mode
        shooter.setPower(.5); // set power for encoder mode
    }


    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        /*
        // Controls block
        if (gamepad2.dpad_up || gamepad1.dpad_up) { // increment shooter tick position by 25 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() + 25);
        }
        if (gamepad2.dpad_down || gamepad1.dpad_down) { // decrement shooter tick position by 25 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() - 25);
        }

        if (gamepad2.start || gamepad1.start) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Telemetry block
        // telemetry.addData("loader", loader.getPosition()); //.15 at rest to .5
        // telemetry.addData("beaconPress", beaconPress.getPosition());
        telemetry.addData("Red", redBlueSensor.red());
        telemetry.addData("Blue", redBlueSensor.blue());
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
        telemetry.addData("White Line", odSensor.getLightDetected());
        */
    }
}
