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

//import android.companion.DeviceFilter;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.net.HttpURLConnection;

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

    boolean elevatorFlag = false;//电梯位置标志
    boolean triggerFlag = false;//
    boolean lbFlag = false,rbFlag = false;//左右bumper是否按下标志

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //连接imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        //等待响应
        //while (imu.isGyroCalibrated());
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

        double speedModify;//pid修正速度

        //-------------------电梯参数---------------------------------------------------------------
        final double MAX_POS_left     =  0.65;     // Maximum rotational position
        final double MAX_POS_right     =  0.33;     // Maximum rotational position
        final double MIN_POS_left     =  0.33;     // Minimum rotational position
        final double MIN_POS_right     =  0.6;     // Minimum rotational position
        //-----------------------------------------------------------------------------------------

        //-------------------------------发射推杆---------------------------------------------------
        final double TRI_RUN = 0.9;
        final double TRI_STOP =0.5;
        //-----------------------------------------------------------------------------------------

        //----------------------------目标角跟随参数-------------------------------------------------
        final double angleIncreaseCoefficient = 1;
        //-----------------------------------------------------------------------------------------

        /*
        //----------------------------计算发射坡度部分-----------------------------------------------
        final double carHeight = 0.3;//车高
        final double shootHeight = 1.0;//篮筐高度
        final double g_v2 = 1.0;//重力除以(发射速度的平方)
        final double angleToPosition = 0.5;//角度转换成舵机位置

        double distance=0;//传感器获取距离
        double slopeAngle, calculateAssistAngle;
        //计算斜坡需要的角度
        calculateAssistAngle = Math.atan((carHeight-shootHeight)/distance);
        slopeAngle = (distance*distance*g_v2+shootHeight-carHeight)/Math.sqrt(distance*distance+(shootHeight-carHeight)*(shootHeight-carHeight));
        slopeAngle = Math.asin(slopeAngle)-calculateAssistAngle;
        slopeAngle = Math.toDegrees(slopeAngle/2);
        slope.setPosition(slopeAngle*angleToPosition);
        //--------------------------------发射坡度调整结束-------------------------------------------
        */

        //------------------------获取部分按键值---------------------------------------------------
        y            = -gamepad1.left_stick_y ;
        x            = gamepad1.right_stick_x ;
        lt           = gamepad1.left_trigger;
        rt           = gamepad1.right_trigger;
        intakePower  =-gamepad2.left_stick_y;
        //-------------------------获取结束-----------------------------------------------------

        //----设定pid修正的目标角，调整‘angleIncreaseCoefficient’使其与车子旋转角度基本一致--------------
        if (targetAngle < 160)
            targetAngle += angleIncreaseCoefficient * lt;
        else
            lt = 0;
        if (targetAngle> -160)
            targetAngle -= angleIncreaseCoefficient * rt;
        else
            rt = 0;
        //-------------------------------目标角调整结束----------------------------------------------

        //---------------------------按lb和rb进行角度微调，按一下调15度。------------------------------
        if (gamepad1.left_bumper) {
            if(lbFlag == false){
                lbFlag = true;
                targetAngle += 15;
            }
        }
        else
            lbFlag = false;
        if (gamepad1.right_bumper){
            if(rbFlag == false){
                rbFlag = true;
                targetAngle -= 15;
            }
        }
        else
            rbFlag = false;
        //-------------------------------------微调结束--------------------------------------------

        //-----------------------------------pid获取修正速度--------------------------------------
        speedModify=gyroModifyPID(targetAngle);
        //-------------------------------------修正结束-------------------------------------------

        //-----------------调整至绝对角度：0°（按x）和15°（按b），便于发射------------------------------
        if (gamepad1.x)
            targetAngle = 15;
        if (gamepad1.b)
            targetAngle = 0;
        //-----------------------------------------结束---------------------------------------------

        //---------------------------------设定轮子速度----------------------------------------------
        LeftFront.setPower( ((y+x)+(rt-lt)) - speedModify );
        LeftRear.setPower( ((y-x)+(rt-lt)) - speedModify);
        RightFront.setPower( ((y-x)+(lt-rt)) + speedModify);
        RightRear.setPower( ((y+x)+(lt-rt)) + speedModify);
        intake.setPower(intakePower);
        //---------------------------------速度设定结束----------------------------------------------

        //--------------------------------发射相关操作-----------------------------------------------
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
        //-----------------------------------发射结束-----------------------------------------------

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", y, x);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", lt, rt);

        telemetry.update();
    }

    /**
     * **************************gyroModifyPID***********************************
     * 根据目标角，返回速度修正
     * 将右边电机速度加上修正值，左边减去修正值
     *
     * 修正后的效果为：车头指向角度targetAngle
     * 调整targetAngle可以实现转向
     * ***************************************************************************
    * */
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
