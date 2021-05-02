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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@TeleOp(name="右！", group="Iterative Opmode")

public class Manual_Right extends OpMode
{
    //-------------------------------电机和传感器声明们--------------------------------------------
    final ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightRear = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;
    private Servo intakeTrigger;
    Servo trigger;
    Servo elevatorLeft;
    Servo elevatorRight;
    Servo slope;
    Servo triggerRoller;
    Servo clamp;
    DcMotor arm;

    private DigitalChannel touchSensor0;
    private DigitalChannel touchSensor1;
    private TouchSensor touchBehind;
    private TouchSensor touchUpon;
    BNO055IMU imu;
    //--------------------------------声明结束------------------------------------------------------

    double targetAngle = 0;
    double angleOfSlope = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        elevatorFlag = triggerFlag = false;
        targetAngle = 0;
        lastSideEncoder = lastMidEncoder = 0;
        position_x = initX * perRound / circumference;
        position_y = initY * perRound / circumference;

        //定义imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //延时0.5秒，以确保imu正常工作
        try {
            Thread.sleep(500);//单位：毫秒
        } catch (Exception e) {
        }

        //定义电机
        LeftFront     = hardwareMap.get(DcMotor.class, "leftfront");
        LeftRear      = hardwareMap.get(DcMotor.class, "leftrear");
        RightFront    = hardwareMap.get(DcMotor.class, "rightfront");
        RightRear     = hardwareMap.get(DcMotor.class, "rightrear");
        intake        = hardwareMap.get(DcMotor.class, "intake");
        shooter       = hardwareMap.get(DcMotor.class, "shooter");
        intakeTrigger = hardwareMap.get(Servo.class, "intaketrigger");
        trigger       = hardwareMap.get(Servo.class, "trigger");
        elevatorLeft  = hardwareMap.get(Servo.class, "elevatorleft");
        elevatorRight = hardwareMap.get(Servo.class, "elevatorright");
        slope         = hardwareMap.get(Servo.class, "slope");
        triggerRoller = hardwareMap.get(Servo.class, "triggerroller");
        clamp         = hardwareMap.get(Servo.class,"clamp");
        arm       = hardwareMap.get(DcMotor.class,"arm");
        touchSensor0 = hardwareMap.get(DigitalChannel.class, "touchsensor0");
        touchSensor1 = hardwareMap.get(DigitalChannel.class, "touchsensor1");
        touchBehind = hardwareMap.get(TouchSensor.class,"touchBehind");
        touchUpon = hardwareMap.get(TouchSensor.class,"touchUpon");

        //设置方向
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        triggerRoller.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.FORWARD);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        touchSensor0.setMode(DigitalChannel.Mode.INPUT);
        touchSensor1.setMode(DigitalChannel.Mode.INPUT);
        //----------------------------编码器初始化---------------------------------------------------
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //------------------------------------------------------------------------------------------

        imu.initialize(parameters);
        //延时0.5秒，以确保imu正常工作
        try {
            Thread.sleep(500);//单位：毫秒
        } catch (Exception e) {
        }

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
        armOption();
        clampOption();
        positionUpdate();//实时坐标位置数据更新
        angleControl();//角度调整

        double speedModify = gyroModifyPID(targetAngle);//获取pid修正速度
        //------------------------------手柄设定轮子速度---------------------------------------------
        float y = -gamepad1.left_stick_y;
        float x = gamepad1.right_stick_x;
        //设定轮子速度
        LeftFront.setPower(y + x- speedModify);
        LeftRear.setPower(y - x - speedModify);
        RightFront.setPower(y - x + speedModify);
        RightRear.setPower(y + x + speedModify);
        intake.setPower(gamepad2.left_stick_y);
        //---------------------------------速度设定结束----------------------------------------------

        shootOperation();//发射相关操作

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /**
     * *****************          gyroModifyPID           ************************
     * 
     * 功能描述：根据目标角，返回速度修正。
     *          修正后的效果为：车头指向角度targetAngle。
     * 使用方法：1. 将右边电机速度加上修正值，左边减去修正值。
     *          2. 调整targetAngle可以实现转向。
     * ***************************************************************************
    * */
    double proportionAngle = 0,integralAngle = 0,differentiationAngle = 0;//pid变量
    double lastAngleErr;
    double Kp = 0.45,Ki = 0.0,Kd = 0.04;//pid参数
    public double gyroModifyPID(double targetAngle) {
        Orientation angle;
        angle =imu.getAngularOrientation();
        proportionAngle=targetAngle-angle.firstAngle;//计算角度误差，作为比例误差
        differentiationAngle=proportionAngle-lastAngleErr;//微分
        integralAngle=integralAngle+proportionAngle;//积分
        lastAngleErr=proportionAngle;
        telemetry.addData("angle","%.3f",angle.firstAngle);
        return (Kp*proportionAngle+Ki*integralAngle+Kd*differentiationAngle) / 15;
    }

    /**
     * **********************         positionUpdate             **********************
     * 
     * 功能描述：根据编码器和陀螺仪的值，更新当前坐标，并输出到屏幕
     *          以赛场左下角为原点，向右为x正方向，向前为y正方向
     * 使用方法：使用方法: 直接放进loop中
     * ********************************************************************************
     * */
    //编码器参数
    final int perRound = 4096;
    final double diameter = 3.8;
    final double circumference = diameter * Math.PI;
    final double initX = 347.26, initY = 31;
    double position_x = initX * perRound / circumference,
            position_y = initY * perRound / circumference;
    int lastSideEncoder = 0, lastMidEncoder = 0;
    public void positionUpdate(){
        if(gamepad1.x) {
            init();
        }
        //获取编码器和陀螺仪值
        int sideEncoder = (RightFront.getCurrentPosition() + LeftFront.getCurrentPosition())/2;
        int midEncoder = RightRear.getCurrentPosition();
        double angle = imu.getAngularOrientation().firstAngle;

        double deltaSide =( sideEncoder - lastSideEncoder);
        double deltaMid = (midEncoder - lastMidEncoder);
        double sinA = Math.sin(Math.toRadians(angle));
        double cosA = Math.cos(Math.toRadians(angle));

        position_x += deltaSide * sinA;
        position_x += deltaMid * cosA;
        position_y -= deltaSide * cosA;
        position_y += deltaMid * sinA;

        lastMidEncoder = midEncoder;
        lastSideEncoder = sideEncoder;
        telemetry.addData("position(cm)","x:%.2f ,y:%.2f",
                position_x / perRound * circumference,position_y / perRound * circumference);
    }

    /**
     * ************************      autoAim       ************************************
     * 
     * 功能描述：自动瞄准，并调整斜坡位置
     *          根据当前车子坐标和目标位置的坐标，调整targetAngle进行瞄准
     *          调用slopeModify进行斜坡调整
     * 使用方法：直接调用, 参数为枚举对象TargetObject
     * 函数关系式:
     *          *
     * 注意: 1. 瞄准只改变targetAngle,没转向,所以要等转向完毕后再按发射键
     *      2. 为避免重复运算,按下瞄准键后只进行一次调用
     *      3. 目标位置的坐标值单位是厘米(cm)
     *      4. 传给slopeModify的参数的单位是米(m)
     * *******************************************************************************
     * */
    public class TargetObject{
        double x, y, height;
        public TargetObject(double co_x, double co_y, double co_z){
            this.x = co_x;
            this.y = co_y;
            this.height = co_z;
        }
    }
    TargetObject basketLeft = new TargetObject(91.44,365.76, 0.996375);
    TargetObject firstPoleLeft = new TargetObject(131.445,365.76, 0.78184375);
    TargetObject secondPoleLeft = new TargetObject(151.765,365.76, 0.78184375);
    TargetObject thirdPoleLeft = new TargetObject(172.085,365.76, 0.78184375);
    TargetObject basketRight = new TargetObject(274.32,365.76, 0.996375);
    TargetObject firstPoleRight = new TargetObject(193.675,365.76, 0.78184375);
    TargetObject secondPoleRight = new TargetObject(213.995,365.76, 0.78184375);
    TargetObject thirdPoleRight = new TargetObject(234.315,365.76, 0.78184375);
    public double autoAim(TargetObject target){
        final double shootAngleModify =
                Math.toDegrees(Math.acos( 0.994337680164722 ));
        //首先计算当前位置与目标坐标连线的和y轴正方向(陀螺仪0°角)夹角,并设定

        double relative_x = target.x - position_x / perRound * circumference;
        double relative_y = target.y - position_y / perRound * circumference;
        targetAngle = -Math.toDegrees(Math.atan(relative_x/relative_y)) - shootAngleModify;
        //斜坡(发射角度)调整
        double distance = Math.sqrt(relative_x*relative_x + relative_y*relative_y);
        return slopeModify(distance / 100, target);
    }

    /**
     * ************************         slopeModify       ****************************
     * 
     * 功能描述：根据离篮筐的距离，计算所需的发射角度，调整slope角度。
     * 使用方法：直接调用,参数为距离。
     * 2021-4-27参数测定:
     *                  初始角18°
     *                  0.6 - 18
     *                  0.7 - 26
     *                  0.8 - 34
     *                  position = - 0.1 / 8 * angle + 0.825
     *注意事项: 1. 为了使表达式简单,本函数使用单位米(m)
     * ******************************************************************************
     * */
    public double slopeModify(double distance,TargetObject tar){
        final double carHeight = 0.24;//车高
        final double g_v2 = 0.0168328914699322;//   重力/(发射速度的平方)
        final double angleToPosition = -0.0145, compensate = 0.825;//角度转换成舵机位置的线性关系系数
        //计算斜坡需要的角度
        double slopeAngle, calculateAssistAngle;
        calculateAssistAngle = Math.atan((carHeight-tar.height)/distance);
        slopeAngle = (distance*distance*g_v2 + tar.height - carHeight)
                / Math.sqrt(distance*distance + (tar.height-carHeight)*(tar.height-carHeight));
        slopeAngle = Math.asin(slopeAngle) - calculateAssistAngle;
        slopeAngle = Math.toDegrees(slopeAngle / 2);

        slope.setPosition(slopeAngle * angleToPosition + compensate);
        telemetry.addData("s position","%.3f",slopeAngle * angleToPosition + compensate);
        return slopeAngle;
    }

    /**
     * ************************          angleControl       *************************
     * 
     * 功能描述: 设定pid修正的目标角,进行角度调整
     *          按下lt和rt，大调
     *          按下lb和rb，小调，按一次调5°
     * 注意事项: 
     *          角度旋转会略有延迟,并且手感不是线性的
     *          原因: 旋转实际上是调整pid的目标角, 并没有直接使用轮子进行旋转操作,
     *          而是使用pid将其修正到设定位置上
     * 使用方法: 直接放进loop中
     * ******************************************************************************
     */
    boolean lbFlag = false,rbFlag = false;//左右bumper是否按下标志，以确保每次按下之后只调整一次角度
    public void angleControl(){
                //------------大调,按下lt和rt进行旋转，调整‘angleIncreaseCoefficient’以改变其速度---------------
                final double angleIncreaseCoefficient = 3.5;
                if (targetAngle < 160)
                    targetAngle += angleIncreaseCoefficient * gamepad1.left_trigger;
        
                if (targetAngle> -160)
                    targetAngle -= angleIncreaseCoefficient * gamepad1.right_trigger;
                //----------------------小调,按lb和rb进行角度微调，按一下调5度。-------------------------------
                if (gamepad1.left_bumper) {
                    if(!lbFlag){
                        lbFlag = true;
                        targetAngle += (targetAngle < 160) ? 5 : 0;
                    }
                }
                else
                    lbFlag = false;
                if (gamepad1.right_bumper){
                    if(!rbFlag){
                        rbFlag = true;
                        targetAngle -= (targetAngle> -160) ? 5 : 0;
                    }
                }
                else
                    rbFlag = false;
                //------------------------------调整至绝对角度：0°-------------------------------------------
                if (gamepad1.b)
                    targetAngle = 0;
    }

    /**
     * *************************          shootOperation       **************************************
     * 功能描述: 发射相关操作, 包括:
     *          1. 自动瞄准: 按up瞄篮筐; 按left\down\right分别瞄第一\二\三个杆
     *          2. 如果开始时trigger没有顶到底,会自动复位
     *          3. 电梯升起\电梯降下\发射(只有在电梯升起时才会发射)
     * 使用方法: 直接放进loop中
     */
    boolean elevatorFlag = false;//电梯位置标志
    boolean triggerFlag = false;//是否复位标志
    boolean aimFlag = false;//是否按下瞄准键标志
    public void shootOperation(){
        //------------------------------电梯参数-----------------------------------------------------
        final double MAX_POS_left     =  0.65;
        final double MAX_POS_right    =  0.33;
        final double MIN_POS_left     =  0.33;
        final double MIN_POS_right     =  0.6;
        //-----------------------------发射推杆参数--------------------------------------------------
        final double TRI_RUN = 0.9;
        final double TRI_STOP =0.5;
        //------------------------------------------------------------------------------------------

        //---------------------------------------瞄准-----------------------------------------------
        if (gamepad1.dpad_up) {
            if(!aimFlag) {
                aimFlag = true;
                angleOfSlope = autoAim(basketRight);
            }
        }
        else if(gamepad1.dpad_left){
            if(!aimFlag) {
                aimFlag = true;
                angleOfSlope = autoAim(firstPoleRight);
            }
        }
        else if (gamepad1.dpad_down){
            if(!aimFlag) {
                aimFlag = true;
                angleOfSlope = autoAim(secondPoleRight);
            }
        }
        else if (gamepad1.dpad_right){
            if(!aimFlag) {
                aimFlag = true;
                angleOfSlope = autoAim(thirdPoleRight);
            }
        }
        else aimFlag = false;
        telemetry.addData("slope","%.3f",angleOfSlope);
        //-----------------------------------------------------------------------------------------

        //------------------------------------trigger复位-------------------------------------------
        if (!triggerFlag) {
            if(!elevatorFlag) {
                elevatorLeft.setPosition(MAX_POS_left);
                elevatorRight.setPosition(MAX_POS_right);
            }
            while (true) {
                if (touchSensor1.getState())
                    trigger.setPosition(TRI_RUN);
                if (touchSensor0.getState()) {
                    trigger.setPosition(TRI_STOP);
                    if(!elevatorFlag) {
                        elevatorLeft.setPosition(MIN_POS_left);
                        elevatorRight.setPosition(MIN_POS_right);
                    }
                    break;
                }
            }
            triggerFlag = true;
        }
        //-----------------------------------复位结束------------------------------------------------

        if (!elevatorFlag) {//电梯在下面时按下Y，则升起电梯
            if(gamepad2.y) {
                elevatorLeft.setPosition(MAX_POS_left);
                elevatorRight.setPosition(MAX_POS_right);
                elevatorFlag = true;
                shooter.setPower(1);
                triggerRoller.setPosition(0.9);
            }
        }
        else {
            while (gamepad2.right_bumper) {//电梯在上面时按RB，发射
                trigger.setPosition(TRI_RUN);
                triggerFlag = false;
            }
            if (gamepad2.a) {//电梯在上面时按A，下降
                elevatorLeft.setPosition(MIN_POS_left);
                elevatorRight.setPosition(MIN_POS_right);
                shooter.setPower(0);
                triggerRoller.setPosition(0.5);
                elevatorFlag = false;
            }
        }
    }

    /**
     * ********************************临时函数testSlope*********************************************
     * 功能描述: 通过按下rb和lb;rt和lt,调整position = a * angle + b的系数
     * *********************************************************************************************
     * */
    double a = -0.0125, b = 0.825;
    boolean lbF = false, rbF = false;
    public void testSlope(double angle){
        double position = a * angle + b;
        if(gamepad2.right_bumper) {
            if(!rbF) {
                rbF = true;
                a += 0.0005;
            }
        } else if(gamepad2.right_trigger > 0) {
            if (!rbF) {
                rbF = true;
                b += 0.005;
            }
        }
        else rbF = false;

        if(gamepad2.left_bumper) {
            if(!lbF) {
                lbF =true;
                a -= 0.0005;
            }
        }else if(gamepad2.left_trigger > 0){
            if(!lbF) {
                lbF =true;
                b -= 0.005;
            }
        }
        else lbF = false;
        slope.setPosition(position);
        telemetry.addData("a","%.4f",a);
        telemetry.addData("b","%.4f",b);
    }

    /**
     * **************************        clampOption         ***************************************
     * 功能描述: 按一次键，架子加紧/放下
     * 使用方法: 直接放进loop
     * *********************************************************************************************
     * */
    boolean clampFlag = false;
    boolean buttonFlag = false;
    public void clampOption() {
        final double CLOSE = 0.26;
        final double OPEN = 0.7;
        if (gamepad2.dpad_down) {
            if (!buttonFlag) {
                buttonFlag = true;
                clampFlag = !clampFlag;
                if (clampFlag)
                    clamp.setPosition(OPEN);
                else
                    clamp.setPosition(CLOSE);
            }
        } else buttonFlag = false;
    }

    /**
     * **************************        armOption           ***************************************
     * 功能描述: 按一次键，机械臂抬起/下降
     * 使用方法: 直接放进loop
     * *********************************************************************************************
     * */
    boolean armFlag = false;
    public void armOption(){
        if(gamepad2.dpad_up) {
            armFlag = !armFlag;
            if (armFlag) {
                arm.setPower(0.8);
                while (!touchUpon.isPressed())
                    ;
                arm.setPower(0);
            } else {
                arm.setPower(-0.8);
                while (!touchBehind.isPressed())
                    ;
                arm.setPower(0);
            }
        }
    }
    //按下停止
    @Override
    public void stop() {
    }
}
