package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Ultra on 9/29/2016
 * Edited by Jonathan on 11/4/2016
 * Edited by Banks on 11/4/2016
 * QUALIFIER 1: Duct Ties & Zip Tape
 * Edited by George on 11/5/2016
 * Edited by Chris on 11/5/2016
 * Edited by Banks on 11/5/2016
 *
 * Edited by Banks on 11/9/2016
 */

public class BaseOp extends OpMode {

    // Controllers
    DcMotorController frontController; // Motor Controller 1
    DcMotorController backController; // Motor Controller 2
    DcMotorController shooterController; // Shooter Controller TODO: change this in robot config to "Shooter Controller"
    ServoController servoController; // Servo Controller 1
    DeviceInterfaceModule cdi; // Core Device Interface Module

    // Motors
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter; // catapult motor
    DcMotor collector; // spinny foam intake

    // Servos
    Servo loader; // shooter loader servo
    Servo beaconPress; // beacon presser servo

    // Sensors

    ColorSensor redBlueSensor; // Adafruit RGBW sensor
    OpticalDistanceSensor odSensor; // Modern Robotics RGBW sensor
    GyroSensor gyro; // Gyro

    // Variables
    public int shooterTargetPosition = 0;
    public boolean shooting = false;
    public boolean loading = false;

    public void init() { // runs when any OpMode is initalized, sets up everything needed to run robot

        // Controller/Module block
        frontController = hardwareMap.dcMotorController.get("Motor Controller 1"); // serial AL00VXVX
        backController = hardwareMap.dcMotorController.get("Motor Controller 2"); // serial AL00VXRV
        shooterController = hardwareMap.dcMotorController.get("Shooter Controller"); // TODO: Get serial and put it here

        servoController = hardwareMap.servoController.get("Servo Controller 1"); // TODO: Get serial and put it here

        cdi = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"); // TODO: Get serial and put it here
        //cdi.setLED(1, true); // turn on Blue LED TODO: does this work?


        // Motor block
        shooter = hardwareMap.dcMotor.get("shooter"); // Shooter Motor
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector = hardwareMap.dcMotor.get("collector"); // Spinny Foam Thing Motor
        collector.setDirection(DcMotor.Direction.REVERSE);
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Servo block
        loader = hardwareMap.servo.get("loader"); // shooter loader servo
        loader.setPosition(0.0); // TODO: Find right value for this

        beaconPress = hardwareMap.servo.get("beaconPress"); // beacon presser servo
        beaconPress.setPosition(90.0); // TODO: Find right value for this

        // TODO: enableLED doesn't seem to work, why?
        // Sensor block
        redBlueSensor = hardwareMap.colorSensor.get("redBlueSensor");

        odSensor = hardwareMap.opticalDistanceSensor.get("odSensor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate(); // calibrate gyro on init TODO: How long does this take?
    }

    @Override
    public void init_loop() { // runs after pressing INIT and loops until START pressed
        super.init_loop();

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // TODO: Experiment with different increments
        if (gamepad2.dpad_up) { // bring shooter up by 25 ticks
            shooter.setTargetPosition(shooter.getCurrentPosition() + 25);
        }
        if (gamepad2.dpad_down) { // bring shooter down by 25 ticks
            shooter.setTargetPosition(shooter.getCurrentPosition() - 25);
        }

        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        telemetry.addData("Shooter Position", shooter.getCurrentPosition());
        telemetry.update();
    }

    public void setAutoRunMode() { // sets things specific to autonomous
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION); // sets the encoders to the right mode, tells the motor to run to the position it is given
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTeleRunMode() { // sets things specific to teleop
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // tells the motors to use encoders
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        beaconPress.setPosition(.5);
    }

    public void loop() { // constantly running code
        telemetry.addData("ShooterEncoderTarget", shooterTargetPosition);
        telemetry.addData("ShooterEncoderPosition", shooter.getCurrentPosition());
        updateTelemetry(telemetry);

    }

    public boolean shooterReady() { // DO NOT TOUCH
        return shooter.getCurrentPosition() <= shooterTargetPosition + 10; // DO NOT TOUCH
    }

    // TODO: Are there any improvements we can make here?
    public void shootParticle() { // Teleop particle shooting
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!shooting && shooterReady()) { // if NOT SHOOTING but READY TO SHOOT
            shooterTargetPosition -= 3360; // make our target to 1 full rotation
            shooter.setTargetPosition(shooterTargetPosition); // set that target as above
            shooting = true;
            loading = false;
            telemetry.addData("getting ready to shoot", "");
        }
        else if (shooting && shooterReady() && !loading) { // IF SHOOTING and READY TO SHOOT
            loading = true;
            loader.setPosition(0.5);
            telemetry.addData("Trying to Load", "");
        }
        else if (loading && loader.getPosition() <= 0.49) { // IF LOADING and LOADER NOT THERE
            loading = false;
            shooting = false;
            loader.setPosition(0.15);
            telemetry.addData("else done shooting", "");
        }
        else {
            shooting = false;
            loading = false;
            telemetry.addData("None of the Above", "");
        }
    }
    protected void stopDriving(){
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }

    protected void MecanumGamepadDrive() {
        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double direction = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rotation = gamepad1.right_stick_x;
        MecanumDrive(speed, direction, rotation);
    }

    protected void MecanumDrive(double speed, double direction, double rotation) {
        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }


}


