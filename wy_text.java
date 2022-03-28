package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "测试_wy",group = "aba_aba")

public class wy_text extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private WyMoveMethod carChassis = new WyMoveMethod();
    private SuperStructures structures = new SuperStructures();

    @Override
    public void init() {
        carChassis.init(hardwareMap, telemetry, gamepad1, gamepad2);
        carChassis.imuInit("imu");
        carChassis.motorInit("leftFront", "leftRear", "rightFront", "rightRear");
        structures.init(hardwareMap, telemetry, gamepad1, gamepad2);
        structures.motorInit("duck", "armLift", "intake", "touch");
        structures.servoInit("Arm", "Claw", "Door");

        DcMotor encoder = hardwareMap.get(DcMotor.class, "encoder");
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        carChassis.extraEncoderInit(encoder, structures.get_right_encoder(), structures.get_mid_encoder());

        structures.claw_arm_Reset();
        telemetry.addData("state", "初始化好了");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("dir", "");
        carChassis.basicMoveThroughGamePad();
        carChassis.showIMU();
        carChassis.showEncoders();
        structures.showEncoder();
        structures.optionThroughGamePad();
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("stop", "stop");
        telemetry.update();
    }
}
