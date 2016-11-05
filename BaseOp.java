package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Ultra on 9/29/2016.
 * Edited by Jonathan on 11/4/2016
 * Edited by Banks T on 11/4/2016.
 */

public class BaseOp extends OpMode {

    // Controllers
    DcMotorController frontController;
    DcMotorController backController;
    DcMotorController shooterController;
    ServoController servoController;
    DeviceInterfaceModule cdi;

    // Motors
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor shooter;
    DcMotor collector;

    // Servos
    Servo loader;
    Servo buttonPress;

    // Sensors
    ColorSensor redBlueSensor; // Adafruit RGBW sensor
    //ColorSensor odSensor; // Modern Robotics RGBW sensor
    GyroSensor gyro; // TODO: This is never initialized below

    // Variables
    public int shooterTargetPosition = 0;
    public boolean shooting = false;
    public boolean loading = false;
    public int particlesShot = 0;

    public void init() { // runs when any OpMode is initalized, sets up everything needed to run robot

        // Controller block
        frontController = hardwareMap.dcMotorController.get("Motor Controller 1"); // serial AL00VXVX

        backController = hardwareMap.dcMotorController.get("Motor Controller 2"); // serial AL00VXRV

        shooterController = hardwareMap.dcMotorController.get("Shooter controller"); //

        servoController = hardwareMap.servoController.get("Servo Controller 1");

        cdi = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        //cdi.setLED(1, true); // turn on Blue LED


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
        loader = hardwareMap.servo.get("loader");
        loader.setPosition(0.0);

        buttonPress = hardwareMap.servo.get("buttonPress");
        buttonPress.setPosition(90.0);


        // Sensor block
        redBlueSensor = hardwareMap.colorSensor.get("redBlueSensor");
        //redBlueSensor.enableLed(false); // disable LED by default

        //odSensor = hardwareMap.colorSensor.get("odSensor");
        //odSensor.enableLed(false); // disable LED by default
        // TODO: Initialize ODS and Gyro

        //gyro = hardwareMap.gyroSensor.get("Gyro");

    }

    @Override
    public void init_loop() {
        super.init_loop();
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (gamepad2.a) {
            shooter.setTargetPosition(shooter.getCurrentPosition() + 25);
        }
        if (gamepad2.y) {
            shooter.setTargetPosition(shooter.getCurrentPosition() - 25);
        }

        if (gamepad1.start || gamepad2.start) { // zero encoder when 1 or 2 presses Start
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // TODO: Use gamepad buttons to move shooter one way or another and reset encoders

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
        buttonPress.setPosition(.5);
    }

    public void ResetShooter() {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Zero encoder
    }

    public void loop()// constantly running code
    {
        telemetry.addData("ShooterEncoderTarget", shooterTargetPosition);
        telemetry.addData("ShooterEncoderPosition", shooter.getCurrentPosition());
        updateTelemetry(telemetry);

    }

    public boolean shooterReady() {
        return shooter.getCurrentPosition() <= shooterTargetPosition + 10; // DO NOT TOUCH
    }

    public void shootParticle() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!shooting && shooterReady()) { // if NOT SHOOTING but READY TO SHOOT
            shooterTargetPosition -= 3360;  // TODO: Reverse motor and go positive?
            shooter.setTargetPosition(shooterTargetPosition);
            shooting = true;
            loading = false;
            // loader.setPosition(.15);
            telemetry.addData("getting ready to shoot", "");
        } else if (shooting && shooterReady() && !loading) { // IF SHOOTING and READY TO SHOOT
            loading = true;
            loader.setPosition(0.5);
            telemetry.addData("Trying to Load", "");
        } else if (loading && loader.getPosition() <= 0.49) { // IF LOADING and LOADER NOT THERE
            loading = false;
            shooting = false;
            loader.setPosition(0.15);
            telemetry.addData("else done shooting", "");
        } else {
            shooting = false;
            loading = false;
            telemetry.addData("None of the Above", "");
        }

    }

    public void loadParticle() {
        loader.setPosition(.5);
    }

    public void auto2Ball() {
        shooter.setPower(1.0);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setTargetPosition(-3360); // Shoots 1 ball
        particlesShot = 1; // we have shot 1 ball now
        if(shooterReady()) {
            loader.setPosition(0.5);
        }
        loader.setPosition(0.15);
        shooter.setTargetPosition(shooterTargetPosition - 3360);

    }
}
