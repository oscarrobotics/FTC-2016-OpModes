package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by gmana on 2/3/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Servo Test", group = "Oscar")

public class ServoTest extends BaseOp {
    double servoPosition = .5;
    boolean servoWasAdjusted = false;

    @Override
    public void loop() {
        super.loop();
        beaconPress.setPosition(servoPosition);
        if (gamepad2.dpad_up) {
            servoPosition = servoPosition + .01;
        }
        if (gamepad2.dpad_down) { // bring shooter down by 25 ticks
            servoPosition = servoPosition - .01;
        }

        servoWasAdjusted = (gamepad2.dpad_down || gamepad2.dpad_up);
        telemetry.addData("1", servoPosition);
    }
}
