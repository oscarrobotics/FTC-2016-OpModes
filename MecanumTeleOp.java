package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
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
        BeaconPress();
        fullAutoFire();

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


    public void BeaconPress() {
        if (gamepad2.dpad_right) { // if dpad_right pressed
            beaconPress.setPosition(0.0); // move servo to press right button
        } else if (gamepad2.dpad_left) { // else if dpad_left pressed
            beaconPress.setPosition(1.0); // move servo to press left button
        } else {
            beaconPress.setPosition(.6); // move back to static position
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


    public void fullAutoFire() {
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switch (mCurrentState) {
            case STATE_IDLE:
                if(gamepad2.left_trigger > 0.5)
                    newState(State.STATE_LOADING);
                break;

            case STATE_LOADING: // start loading
                timeAtStart = System.currentTimeMillis();
                loader.setPosition(0.5);
                newState(State.STATE_WAIT_TO_SHOOT);
                break;

            case STATE_WAIT_TO_SHOOT: // loading is finished, move back servo for 100ms before firing
                if(System.currentTimeMillis() >= timeAtStart + 500) {
                    loader.setPosition(0.15);
                    newState(State.STATE_INCREMENT_TARGET_POSITION);
                }
                break;

//            case STATE_WAIT_TO_SHOOT:
//                if(System.currentTimeMillis() >= timeAtStart + 1000) {
//                    loader.setPosition(0.2);
//                    shooterTargetPosition -= 3360;
//                    shooter.setTargetPosition(shooterTargetPosition);
//                    newState(State.STATE_INCREMENT_TARGET_POSITION);
//                }
//                break;

            case STATE_INCREMENT_TARGET_POSITION: //
                if(System.currentTimeMillis() >= timeAtStart + 600) {
                    shooterTargetPosition -= 3360;
                    newState(State.STATE_RETURN_TO_IDLE);
                }
                break;

            case STATE_RETURN_TO_IDLE:
                if(shooterReady()) {
                    newState(State.STATE_IDLE);
                }
                break;



        }
    }


    private void newState(MecanumTeleOp.State newState) {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }
}
