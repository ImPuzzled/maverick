package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PIDFarm.PIDF_Arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "MainOpMode")

public class MainOpMode extends OpMode {



    double Speed;
    double MaxSpeed;
    double MinSpeed;
    double MaxTurnSpeed;
    double MinTurnSpeed;
    double TurnSpeed;
    double Vertical;
    double Horizontal;
    double Pivot;
    double pid;
    double ff;
    double power;
    double leftServo;
    double rightServo;
    int newTarget;

    boolean Safety;
    Boolean Armed;

    PIDController controller;

    GamepadEx DriverOp1 = new GamepadEx(gamepad1);
    GamepadEx DriverOp2 = new GamepadEx(gamepad2);

    TriggerReader LTR = new TriggerReader(DriverOp1, GamepadKeys.Trigger.LEFT_TRIGGER);
    TriggerReader RTR = new TriggerReader(DriverOp1, GamepadKeys.Trigger.RIGHT_TRIGGER);
    TriggerReader LTR2 = new TriggerReader(DriverOp2, GamepadKeys.Trigger.LEFT_TRIGGER);
    TriggerReader RTR2 = new TriggerReader(DriverOp2, GamepadKeys.Trigger.RIGHT_TRIGGER);

    DcMotorEx rightFront;
    DcMotorEx leftFront;
    DcMotorEx rightRear;
    DcMotorEx leftRear;

    DcMotorEx armMotor;
    DcMotorEx armMotor2;
    DcMotorEx spoolMotor;

    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    Servo Servo4;
    Servo Servo5;
    Servo Servo6;
    Servo Servo7;


    @Override
    public void init() {
        


        Speed = 0.85;
        MaxSpeed = 1;
        MinSpeed = 0.65;
        TurnSpeed = 0.7;
        MaxTurnSpeed = 0.7;
        MinTurnSpeed = 0.5;
        newTarget = 10;
        Safety = true;
        Armed = false;

        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo3 = hardwareMap.get(Servo.class, "servo 3");
        Servo4 = hardwareMap.get(Servo.class, "servo 4");
        Servo5 = hardwareMap.get(Servo.class, "servo 5");
        Servo6 = hardwareMap.get(Servo.class, "servo 6");
        Servo7 = hardwareMap.get(Servo.class, "servo 7");

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");

        armMotor = hardwareMap.get(DcMotorEx.class, "Launch Motor");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "Launch Motor 2");
        spoolMotor = hardwareMap.get(DcMotorEx.class, "Launch Motor 3");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo1.setPosition(Servo.Direction.REVERSE.ordinal());

        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo3.setPosition(1);
        Servo4.setPosition(0);

        telemetry.addData("Info", "OpMode is ready to run");
        telemetry.update();
    }



    @Override
    public void loop() {

        PIDF_Arm(newTarget);

        if (gamepad2.touchpad && Safety){
            gamepad2.rumble(1000);
            Safety = false;
        } else if (!gamepad2.touchpad && Safety){
            Armed = true;
        } else if (gamepad2.touchpad && Armed){
            Servo5.setPosition(1);
        }

        if (LTR.isDown()){
            Speed = MinSpeed;
            TurnSpeed = MinTurnSpeed;
        } else {
            Speed = MaxSpeed;
            TurnSpeed = MaxTurnSpeed;
        }

        if (gamepad2.left_stick_y > 0){
            newTarget += 2;
        } else if (gamepad2.left_stick_y < 0){
            newTarget -= 2;
        }

        if (DriverOp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            newTarget = 50;
        }
        if (DriverOp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            newTarget = -1350;
        }

        if (LTR2.wasJustPressed()){
            rightServo = (rightServo == 0) ? 1 : 0;
        }
        if (RTR2.wasJustPressed()){
            leftServo = (leftServo == 0) ? 1 : 0;
        }

        if (gamepad2.triangle){
            Servo3.setPosition(0);
            Servo4.setPosition(1);
        }

        Servo1.setPosition(rightServo);
        Servo2.setPosition(leftServo);

        spoolMotor.setPower(gamepad2.right_stick_y);
        Vertical = Math.min(Math.max(gamepad1.left_stick_x, -Speed), Speed);
        Horizontal = Math.min(Math.max(-gamepad1.left_stick_y, -Speed), Speed);
        Pivot = Math.min(Math.max(gamepad1.right_stick_x, -TurnSpeed), TurnSpeed);
        leftRear.setPower(-Pivot + (Vertical - Horizontal));
        rightRear.setPower(-Pivot + Vertical + Horizontal);
        leftFront.setPower(Pivot + Vertical + Horizontal);
        rightFront.setPower(Pivot + (Vertical - Horizontal));
    }

    @Override
    public void stop(){
        telemetry.addData("Info", "OpMode has been successfully stopped");
        telemetry.update();
    }
}