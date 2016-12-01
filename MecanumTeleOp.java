package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.Range;

/**
 * Created by Chris on 11/10/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Mecanum Tank", group = "Oscar")
public class MecanumTeleOp extends BaseOp {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        super.loop();
        MecanumGamepadDrive();

        if (gamepad2.y)
            beaconPress.setPosition(Range.clip( beaconPress.getPosition()+.01, 0.0,1.0));

        if (gamepad2.a)
            beaconPress.setPosition(Range.clip( beaconPress.getPosition()-.01, 0.0,1.0));

        if(gamepad1.x)
            driveSideways();


        Shoot();
        Collect();
        Load();
        BeaconPress();

        telemetry.addData("1", beaconPress.getPosition());
    }
    public void Shoot() {
        if (gamepad2.right_trigger > 0.5) { // If right trigger pressed
            shootParticle(); // fire
        }
    }

    public void Load() {
        if (gamepad2.dpad_down) { // if dpad_down pressed
            loader.setPosition(.5); // load ball
        } else {
            loader.setPosition(0.15); // move back to static position

        }
    }


    public void BeaconPress() {
        if (gamepad2.dpad_right) { // if dpad_right pressed
            beaconPress.setPosition(0.0); // move servo to press right button
        } else if (gamepad2.dpad_left) { // else if dpad_left pressed
            beaconPress.setPosition(1.0); // move servo to press left button
        } else {
            beaconPress.setPosition(.5); // move back to static position
        }
    }

    public void Collect() {
        if (gamepad2.b) { // if B pressed
            collector.setPower(-1.0); // turn off intake
        } else if (gamepad2.a) { // if A pressed
            collector.setPower(1.0); // spin intake outwards
        } else {
            collector.setPower(0.0); // default to off
        }
    }
}