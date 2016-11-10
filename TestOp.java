package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by voelk_000 on 11/2/2016.
 * Edited by Banks T on 11/3/2016.
 */


// TODO: Rename this to something more appropriate, perhaps AlignOp or ZeroOp?
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: TestOp", group = "Oscar")
public class TestOp extends BaseOp {
    @Override
    public void init() {
        super.init();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); // change to position run encoder mode
        shooter.setPower(.5); // set power for encoder mode
    }

    @Override
    public void loop() {

        // Controls block
        // TODO: Fix or re-assign 'loader.setposition' and 'beaconPress.setPosition below'
        loader.setPosition(Range.clip(gamepad1.left_stick_y, 0, 1.0)); // move loader servo with 'left stick y' on gamepad1

        beaconPress.setPosition(Range.clip(gamepad1.right_stick_y, 0, 1.0)); // move beacon servo with 'right stick y' on gamepad1
        // TODO: switch between a fast and slow increment with another button for time-saving
        if (gamepad2.a) { // increment shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() + 10);
        }
        if (gamepad2.y) { // decrement shooter tick position by 10 while held
            shooter.setTargetPosition(shooter.getCurrentPosition() - 10);
        }

        // Telemetry block
        // telemetry.addData("loader", loader.getPosition()); //.15 at rest to .5
        // telemetry.addData("beaconPress", beaconPress.getPosition());
        telemetry.addData("Red", redBlueSensor.red());
        telemetry.addData("Blue", redBlueSensor.blue());
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
    }
}
