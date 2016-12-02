package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ultra on 9/29/2016.
 * Edited by Banks T on 11/9/2016.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Tank", group = "Oscar")
public class TeleOp extends BaseOp {

    public int shooterPos;

    public void init() {
        super.init();
        setTeleRunMode(); // ensure we're set up to use driver control
    }

    @Override
    public void loop() {
        super.loop();

               // Commands
        RobotDrive();
        Shoot();
        Collect();
        Load();
        BeaconPress();

        // Other loop items
        shooterPos = shooter.getCurrentPosition();
        telemetry.addData("ShooterPos", shooterPos);
        telemetry.update(); //sends telemetry

    }

    public void RobotDrive() {

        // Get joystick values
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;

        // Clip off any values that exceed the min & max (-1 to 1)
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // Cube the output for finer control
        leftPower = Math.pow(leftPower, 3);
        rightPower = Math.pow(rightPower, 3);

        // Drive!
        rightFront.setPower(rightPower);
        leftFront.setPower(leftPower);
        rightBack.setPower(rightPower);
        leftBack.setPower(leftPower);
    }


    public void Shoot() {
        if (gamepad2.right_trigger > 0.5) { // If right trigger pressed
            manualFire(); // fire
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
            collector.setPower(0.0); // default to spinning in
        }
    }
}

