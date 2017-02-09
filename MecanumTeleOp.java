package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Chris on 11/10/2016.
 * Edited by George on 12/6/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Mecanum Tank", group = "Oscar")
public class MecanumTeleOp extends BaseOp {

    public long timeAtStart;

    public enum State {
        STATE_IDLE,
        STATE_LOADING,
        STATE_WAIT_TO_SHOOT,
        STATE_LOADER_MOVE_DELAY,
        STATE_INCREMENT_TARGET_POSITION,
        STATE_RETURN_TO_IDLE
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private MecanumTeleOp.State mCurrentState;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        resetStartTime();
        newState(State.STATE_IDLE);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        super.loop();
        MecanumGamepadDrive();
        Shoot();
        Collect();
        Load();
        extendBeaconPress();

        telemetry.addData("1", beaconPress.getPosition());
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

    public void Collect() {
        if (gamepad2.b) { // if B pressed
            collector.setPower(-1.0); // turn off intake
        } else if (gamepad2.a) { // if A pressed
            collector.setPower(1.0); // spin intake outwards
        } else {
            collector.setPower(0.0); // default to off
        }
    }

    private void newState(MecanumTeleOp.State newState) {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }

    private void extendBeaconPress() {
        boolean seeingRed = redBlueSensor.red() > redBlueSensor.blue() + colorSensorMargin;
        boolean seeingBlue = redBlueSensor.blue() > redBlueSensor.red() + colorSensorMargin;
//        if (gamepad2.left_bumper && !leftBumperPressed)
//            if (beaconPress.getPosition() == servoIn) {
//                beaconPress.setPosition(servoExtend);
//                toggleDebug = true;
//            } else if (beaconPress.getPosition() == servoExtend) {
//                beaconPress.setPosition(servoOpposite);
//            } else if (beaconPress.getPosition() == servoOpposite) {
//                beaconPress.setPosition(servoOppositeIn);
//            } else if (beaconPress.getPosition() == servoOppositeIn){
//                beaconPress.setPosition(servoIn);
//                toggleDebug = false;
//            }

        //leftBumperPressed = gamepad2.left_bumper;
        if (gamepad2.x)
            lookingForRed = false;
        if (gamepad2.y)
            lookingForRed = true;

        cdi.setLED(1, seeingRed);
        cdi.setLED(0, seeingBlue);

        boolean correctColor = (lookingForRed && redBlueSensor.red() > redBlueSensor.blue() + colorSensorMargin) ||
                (!lookingForRed && redBlueSensor.blue() > redBlueSensor.red() + colorSensorMargin);

        if (correctColor && beaaconEnabled) {
            beaconPress.setPosition(extendServoPos);
            bringBackInAt = System.currentTimeMillis() + retractDelay;
        } else {
            if (bringBackInAt < System.currentTimeMillis()) {
                beaconPress.setPosition(safeServoPos);
            }
        }
    }
}
