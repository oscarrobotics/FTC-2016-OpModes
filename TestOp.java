package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by voelk_000 on 11/2/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: TestOp", group = "Oscar")

public class TestOp extends BaseOp {
    @Override
    public void init() {
        super.init();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(.5);
    }

    @Override
    public void loop() {
        loader.setPosition(Range.clip(gamepad1.left_stick_y,0,1.0));

        buttonPress.setPosition(Range.clip(gamepad1.right_stick_y,0,1.0));

        if (gamepad2.a){
            shooter.setTargetPosition(shooter.getCurrentPosition() + 10);
        }
        if (gamepad2.y){
            shooter.setTargetPosition(shooter.getCurrentPosition() - 10);
        }


        telemetry.addData("loader", loader.getPosition()); //.15 at rest to .5
       // telemetry.addData("buttonPress", buttonPress.getPosition());
        telemetry.addData("Red", redBlueSensor.red());
        telemetry.addData("Blue", redBlueSensor.blue());
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
    }



}
