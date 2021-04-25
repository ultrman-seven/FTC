/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.companion.DeviceFilter;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="O_0", group="Iterative Opmode")
//@Disabled

public class manual_Iterative_Wy extends OpMode
{
    //声明们
    BNO055IMU imu;
    final ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightRear = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;
    private Servo intaketrigger;
    private DigitalChannel touchsensor0;
    private DigitalChannel touchsensor1;

    Servo trigger;
    Servo elevatorleft;
    Servo elevatorright;
    Servo slope;
    Servo triggerroller;

    double propotionAngle=0,integralAngle=0,differentiationAngle=0;//pid变量
    double lastAngleErr;
    double Kp=0.45,Ki=0.0,Kd=0.04;//pid参数
    double targetAngle = 0;

    boolean elevatorFlag = false;
    boolean triggerFlag = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront     = hardwareMap.get(DcMotor.class, "leftfront");
        LeftRear      = hardwareMap.get(DcMotor.class, "leftrear");
        RightFront    = hardwareMap.get(DcMotor.class, "rightfront");
        RightRear     = hardwareMap.get(DcMotor.class, "rightrear");
        intake        = hardwareMap.get(DcMotor.class, "intake");
        shooter       = hardwareMap.get(DcMotor.class, "shooter");
        intaketrigger = hardwareMap.get(Servo.class, "intaketrigger");
        trigger       = hardwareMap.get(Servo.class, "trigger");
        elevatorleft  = hardwareMap.get(Servo.class, "elevatorleft");
        elevatorright = hardwareMap.get(Servo.class, "elevatorright");
        slope         = hardwareMap.get(Servo.class, "slope");
        triggerroller = hardwareMap.get(Servo.class, "triggerroller");

        touchsensor0 = hardwareMap.get(DigitalChannel.class, "touchsensor0");
        touchsensor1 = hardwareMap.get(DigitalChannel.class, "touchsensor1");
       // motor = hardwareMap.get(DcMotorEx.class,"rightfront");
        //RightFront = hardwareMap.get(DcMotor.class,"rightfront");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        triggerroller.setDirection(Servo.Direction.FORWARD); // set triggerroller direction
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touchsensor0.setMode(DigitalChannel.Mode.INPUT);
        touchsensor1.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //连接imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //等待响应
        while (imu.isGyroCalibrated());
        //imu初始化
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    //按下开始
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float x, y;
        float lt, rt;
        float intakePower;
        double speedModify;

        boolean triggerPosition;

        final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int    CYCLE_MS    =   50;     // period of each cycle
        final double MAX_POS_left     =  0.65;     // Maximum rotational position
        final double MAX_POS_right     =  0.33;     // Maximum rotational position
        final double MIN_POS_left     =  0.33;     // Minimum rotational position
        final double MIN_POS_right     =  0.6;     // Minimum rotational position
        final double TRI_RUN = 0.9;
        final double TRI_STOP =0.5;
        final double angleIncreaseCoefficient = 1;
//        final int padErr = 80;

        y            = -gamepad1.left_stick_y ;
        x            = gamepad1.right_stick_x ;
        lt           = gamepad1.left_trigger;
        rt           = gamepad1.right_trigger;
        intakePower  =-gamepad2.left_stick_y;
        triggerPosition = gamepad2.y;

        if (targetAngle < 160)
            targetAngle += angleIncreaseCoefficient*lt;
        else
            lt = 0;
        if (targetAngle> -160)
            targetAngle -= angleIncreaseCoefficient*rt;
        else
            rt = 0;

        speedModify=gyroModifyPID(targetAngle);

        if (gamepad1.x == true)
            targetAngle = 15;
        if (gamepad1.b == true)
            targetAngle = 0;

        //设定轮子速度
        LeftFront.setPower( ((y+x)+(rt-lt)) - speedModify );
        LeftRear.setPower( ((y-x)+(rt-lt)) - speedModify);
        RightFront.setPower( ((y-x)+(lt-rt)) + speedModify);
        RightRear.setPower( ((y+x)+(lt-rt)) + speedModify);
        intake.setPower(intakePower);

        if (elevatorFlag == false) {
            if (triggerFlag == false) {
                while (touchsensor0.getState() == false)
                    trigger.setPosition(TRI_RUN);
                trigger.setPosition(TRI_STOP);
                triggerFlag = true;
            }
            if (gamepad1.y == true) {
                elevatorleft.setPosition(MAX_POS_left);
                elevatorright.setPosition(MAX_POS_right);
                elevatorFlag = true;
                shooter.setPower(1);
                triggerroller.setPosition(0.9);
            }
        }
        else {
                if (gamepad1.a == true) {
                    elevatorleft.setPosition(MIN_POS_left);
                    elevatorright.setPosition(MIN_POS_right);
                    shooter.setPower(0);
                    triggerroller.setPosition(0.5);
                    elevatorFlag = false;
                }
                while (gamepad1.right_bumper == true) {
                    trigger.setPosition(TRI_RUN);
                    triggerFlag = false;
                }
                if (triggerFlag == false){
                    while (touchsensor0.getState() == false)
                        trigger.setPosition(TRI_RUN);
                    trigger.setPosition(TRI_STOP);
                    triggerFlag = true;
                    }
            }

        //triggerroller.setPosition(0.5);
        /*if (gamepad1.left_stick_y >= 0.3) {
            DCPower +=1;
        } else if (gamepad1.left_stick_y <= -0.3) {
            DCPower += -1;
        } else {
            DCPower += 0;
        }
        if (gamepad1.right_stick_x >= 0.3) {
            DCPower += 1;
        } else if (gamepad1.right_stick_x < -0.3) {
            DCPower += -1;
        } else {
            DCPower += 0;
        }*/
        /*if (gamepad1.y) {
            intaketrigger.setPosition(0.2);
        } else if (gamepad1.a) {
            intaketrigger.setPosition(0.5);
        }
        if (gamepad1.left_bumper) {
            shooter.setPower(1);
        } else if (gamepad1.right_bumper) {
            shooter.setPower(0.1);
        }
        if (gamepad2.x) {
            trigger.setPosition(MIN_POS);
        } else if (gamepad2.b) {
            trigger.setPosition(MAX_POS);
        }
        if (gamepad2.a) {
            elevatorleft.setPosition(MIN_POS);
            elevatorright.setPosition(MAX_POS);
        } else if (gamepad2.y) {
            elevatorleft.setPosition(MAX_POS);
            elevatorright.setPosition(MIN_POS);
        }
        if (gamepad2.left_bumper) {
            slope.setPosition(MIN_POS);
        } else if (gamepad2.right_bumper) {
            slope.setPosition(0.5);
        }
        if (gamepad1.x) {
            triggerroller.setPosition(0.9);
        } else if(gamepad1.b){
            triggerroller.setPosition(0.5);
        }*/


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", y, x);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", lt, rt);

        telemetry.update();
    }

    //根据目标角，返回速度修正
    public double gyroModifyPID(double targetAngle) {
        Orientation angle;
        angle =imu.getAngularOrientation();
        propotionAngle=targetAngle-angle.firstAngle;//计算角度误差，作为比例误差
        differentiationAngle=propotionAngle-lastAngleErr;//微分
        integralAngle=integralAngle+propotionAngle;//积分
        lastAngleErr=propotionAngle;

        return (Kp*propotionAngle+Ki*integralAngle+Kd*differentiationAngle)/15;
    }

    //按下停止
    @Override
    public void stop() {
    }

}
