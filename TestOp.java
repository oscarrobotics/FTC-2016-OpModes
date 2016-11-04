package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by voelk_000 on 11/2/2016.
 * Edited by Banks T on 11/3/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: TestOp", group = "Oscar")

public class TestOp extends BaseOp {
    @Override
    public void init() {
        super.init();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); // change to position closed loop
        shooter.setPower(.5); // ?
    }

    @Override
    public void loop() {

        // Controls block
        loader.setPosition(Range.clip(gamepad1.left_stick_y, 0, 1.0)); // move loader servo with 'left stick y' on gamepad1

        buttonPress.setPosition(Range.clip(gamepad1.right_stick_y, 0, 1.0)); // move beacon servo with 'right stick y' on gamepad1

        if (gamepad2.a) { // increment shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() + 10);
        }
        if (gamepad2.y) { // decrement shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() - 10);
        }

        // Telemetry block
        telemetry.addData("loader", loader.getPosition()); //.15 at rest to .5
        // telemetry.addData("buttonPress", buttonPress.getPosition());
        telemetry.addData("Red", redBlueSensor.red());
        telemetry.addData("Blue", redBlueSensor.blue());
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
    }
}
