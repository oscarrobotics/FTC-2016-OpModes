package org.firstinspires.ftc.teamcode;

/**
 * Created by Chris on 11/10/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Mechanum Tank", group = "Oscar")
public class MecanumTeleOp extends BaseOp {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        MecanumGamepadDrive();
    }

}