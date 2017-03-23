package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Chris on 11/10/2016.
 * Edited by George on 12/6/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Oscar: Teleop Mecanum Tank", group = "Oscar")
public class MecanumTeleOp extends BaseOp {
    enum autoFireStates {FIRE_INIT, FIRE_SHOOT, FIRE_LOAD, FIRE_RETRACT}

    autoFireStates shooterState = autoFireStates.FIRE_INIT;
    private ElapsedTime shooterStateTime = new ElapsedTime();  // Time into current state

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        resetStartTime();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        super.loop();

        boolean seeingRed = redBlueSensor.red() > redBlueSensor.blue() + colorSensorMargin;
        boolean seeingBlue = redBlueSensor.blue() > redBlueSensor.red() + colorSensorMargin;
        cdi.setLED(1, seeingRed);
        cdi.setLED(0, seeingBlue);

        MecanumGamepadDrive();
        Shoot();
        Collect();
        Load();
        extendBeaconPress();
        flipServo();
    }

    private void newShooterState(autoFireStates newState) {
        shooterStateTime.reset();
        shooterState = newState;
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Auto fire target", shooterTargetPosition);
    }

    private void autoFire() {
        switch (shooterState) {
            case FIRE_INIT:
                if (!shooter.isBusy()) {
                    shooterTargetPosition -= shooterRotation; // make our target to 1 full rotation
                    shooter.setTargetPosition(shooterTargetPosition); // set that target as above
                    newShooterState(autoFireStates.FIRE_SHOOT);
                }
                break;
            case FIRE_SHOOT:
                if (!shooter.isBusy()) {
                    loader.setPosition(loaderLoad); // load ball
                    newShooterState(autoFireStates.FIRE_LOAD);
                }
                break;
            case FIRE_LOAD:
                if (shooterStateTime.milliseconds() > 1250) {
                    loader.setPosition(loaderStatic);
                    newShooterState(autoFireStates.FIRE_RETRACT);
                }
                break;
            case FIRE_RETRACT:
                newShooterState(autoFireStates.FIRE_INIT);
                break;
        }
    }

    public void Shoot() {
        if (gamepad2.right_trigger > 0.5) { // If right trigger pressed
            autoFire();
        }
    }

    public void Load() {
        if (shooterState == autoFireStates.FIRE_INIT) {
            if (gamepad2.dpad_down) { // if dpad_down pressed
                loader.setPosition(loaderLoad); // load ball
            } else {
                loader.setPosition(loaderStatic); // move back to static position
            }
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

//    private void newState(MecanumTeleOp.State newState) {
//        // Reset the state time, and then change to next state.
//        mStateTime.reset();
//        mCurrentState = newState;
//    }

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

//        cdi.setLED(1, seeingRed);
//        cdi.setLED(0, seeingBlue);

        boolean correctColor = (lookingForRed && seeingRed) || (!lookingForRed && seeingBlue);

        if (correctColor) {
            beaconPress.setPosition(extendServoPos);
            bringBackInAt = System.currentTimeMillis() + retractDelay;
        } else {
            if (bringBackInAt < System.currentTimeMillis()) {
                beaconPress.setPosition(safeServoPos);
            }
        }

    }

    private void flipServo() {
        if (gamepad1.left_bumper && !servoFlipped) {
            if (safeServoPos == servoOppositeIn) {
                safeServoPos = servoIn;
                extendServoPos = servoExtend;
            } else if (safeServoPos == servoIn) {
                safeServoPos = servoOppositeIn;
                extendServoPos = servoOpposite;
            }
        }

        servoFlipped = gamepad1.left_bumper;
    }
}
