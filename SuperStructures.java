package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicReference;

public class SuperStructures {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad1, gamepad2;

    private DcMotor duck = null;
    private DcMotor armLift = null;
    private DcMotor intake = null;

    private Servo arm = null;
    private Servo claw = null;
    private Servo door = null;

    private TouchSensor touch = null;

    public void init(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2) {
        hardwareMap = h;
        telemetry = t;
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void motorInit(String duck, String arm, String intake, String touch) {
        this.duck = hardwareMap.get(DcMotor.class, duck);
        this.armLift = hardwareMap.get(DcMotor.class, arm);
        this.intake = hardwareMap.get(DcMotor.class, intake);
        this.touch = hardwareMap.get(TouchSensor.class, touch);
        this.duck.setDirection(DcMotor.Direction.FORWARD);
        this.armLift.setDirection(DcMotor.Direction.REVERSE);
        this.intake.setDirection(DcMotor.Direction.FORWARD);

        this.duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!this.touch.isPressed())
            this.armLift.setPower(-0.2);
        this.armLift.setPower(0);
        this.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void servoInit(String arm, String claw, String door) {
        this.arm = hardwareMap.get(Servo.class, arm);
        this.claw = hardwareMap.get(Servo.class, claw);
        this.door = hardwareMap.get(Servo.class, door);
    }

    private double duck_speed = 0.2;
    private int countForDoor = 0;

    public void optionThroughGamePad() {
        float ly = gamepad2.left_trigger - gamepad2.right_trigger;
        if (Math.abs(ly) > 0)
            duck_speed += (duck_speed < 0.99) ? 0.01 : 0;
        else duck_speed = 0.6;
        duck.setPower(duck_speed * duck_speed * ly);

        intake.setPower(gamepad2.left_stick_y);


        if (gamepad2.right_bumper)
            setArmPosition(1500);
        if (gamepad2.b)
            setArmPosition(750);
        if (gamepad2.a) {
            setArmPosition(0);
            door.setPosition(0);
        }

//        valueAdj(gamepad2.dpad_up, this.arm);

        armPositionUpdate();
        final int MAX_Daley_Time = 40;
        if (gamepad2.left_bumper) {
            door.setPosition(0.5);
            countForDoor = 0;
        } else {
            if (countForDoor > MAX_Daley_Time)
                door.setPosition(0);
            else
                countForDoor++;
        }

        clawOperation(gamepad2.dpad_up);

        armOperation(gamepad2.x);

//        arm.setPosition(Math.abs(gamepad2.right_stick_y));
//        telemetry.addData("servo", "%f", gamepad2.right_stick_y);
    }

    public void showEncoder() {
        telemetry.addData("encoder", armLift.getCurrentPosition());
        telemetry.addData("touch sensor", touch.isPressed());
    }

    int arm_encoder_value = 0;

    private void setArmPosition(int val) {
        arm_encoder_value = val;
    }

    private void armPositionUpdate() {
        final int max_position = 1600;
        final int bufVal = 100;
        if (arm_encoder_value < max_position && arm_encoder_value > 5) {
            if (armLift.getCurrentPosition() < arm_encoder_value - bufVal)
                armLift.setPower(0.8);
            else if (armLift.getCurrentPosition() > arm_encoder_value + bufVal)
                armLift.setPower(-0.8);
            else
                armLift.setPower(0);
        } else if (arm_encoder_value == 0) {
            if (armLift.getCurrentPosition() > 210)
                armLift.setPower(-0.8);
            else {
                if (armLift.getCurrentPosition() > 10)
                    armLift.setPower(-0.2);
                while (!touch.isPressed())
                    ;
                armLift.setPower(0);
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public DcMotor get_right_encoder() {
        return duck;
    }

    public DcMotor get_mid_encoder() {
        return intake;
    }

    private void clawOperation(boolean s) {
        if (s)
            claw.setPosition(0.5);
        else claw.setPosition(1);
    }

    int armPutEvent=0;
    boolean keyFlag_arm=false;
    private void armOperation(boolean key) {
        telemetry.addData("X:","%d",armPutEvent);
        if (key) {
            if (!keyFlag_arm) {
                keyFlag_arm = true;
                if (armPutEvent < 3)
                    armPutEvent++;
                else armPutEvent = 0;
            }
            switch (armPutEvent) {
                case 0:
                    arm.setPosition(0);//收回来
                    break;
                case 1:
                    arm.setPosition(0.6);//中间位置
                    break;
                case 2:
                    arm.setPosition(0.7);//最高位置
                    break;
                case 3:
                    arm.setPosition(0.85);//捡的位置
                    break;
            }
        }
        else keyFlag_arm=false;
    }

    public void claw_arm_Reset() {
        clawOperation(false);
        arm.setPosition(0);
    }

    double servoVal = 0;
    boolean pressed = false;
    private void valueAdj(boolean key, Servo s) {
        if (key) {
            if (!pressed) {
                pressed = true;
                servoVal += 0.1;
                s.setPosition(servoVal);
                telemetry.addData("servo", "%f", servoVal);
            }
        } else
            pressed = false;
    }
}
