package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GourmetMode extends OpMode {

    private enum RobotSpeeds {
        SLOW,
        NORMAL,
        FAST
    }
    RobotSpeeds speed = RobotSpeeds.NORMAL;
    double precision = 1.5;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor armMotor;

    Servo rightClaw;
    Servo leftClaw;

    Boolean armIsRaised = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        leftClaw =  hardwareMap.get(Servo.class, "LeftClaw");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_y;
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;

        double FLMotorSpeed = 0;
        double FRMotorSpeed = 0;
        double BLMotorSpeed = 0;
        double BRMotorSpeed = 0;

        telemetry.addData("Gamepad Trigger", rightTrigger);

        if(leftStick != 0 || rightStick != 0) { // Basic forward/back code
            FLMotorSpeed = leftStick;
            BLMotorSpeed = leftStick;

            FRMotorSpeed = rightStick;
            BRMotorSpeed = rightStick;
        } else {
            // dpad right/left is to go sideways
            if(gamepad1.dpad_right) {
                FLMotorSpeed = -1;
                FRMotorSpeed = 1;
                BRMotorSpeed = -1;
                BLMotorSpeed = 1;
            } else if(gamepad1.dpad_left) {
                FLMotorSpeed = 1;
                FRMotorSpeed = -1;
                BRMotorSpeed = 1;
                BLMotorSpeed = -1;
            }
        }

        if (gamepad1.right_bumper) {
            speed = RobotSpeeds.FAST;
        } else if (gamepad1.left_bumper) {
            speed = RobotSpeeds.SLOW;
        } else {
            speed = RobotSpeeds.NORMAL;
        }

        switch(speed) {
            case SLOW:
                precision = 3;
                break;
            case NORMAL:
                precision = 1.5;
                break;
            case FAST:
                precision = 1;
                break;
        }

        frontLeftMotor.setPower(FLMotorSpeed/precision);
        backLeftMotor.setPower(BLMotorSpeed/precision);
        frontRightMotor.setPower(FRMotorSpeed/precision);
        backRightMotor.setPower(BRMotorSpeed/precision);

        if (gamepad1.b) {    // Close claw
            rightClaw.setPosition(0);
            leftClaw.setPosition(1);
        }
        if (gamepad1.x) {    // Open claw
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.5);
        }

        // Arm motor:
        // 0.15 is the power required to keep the arm in place
        // (rightTrigger - leftTrigger/2) is to move the arm up/down respectively
        // Precision is wether precision mode is on
        armMotor.setPower(0.1 + (rightTrigger - leftTrigger/1.5) / precision);
    }
}