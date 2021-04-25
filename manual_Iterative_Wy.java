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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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

@TeleOp(name="A", group="Iterative Opmode")
//@Disabled

public class BasicOpMode_Iterative_WyEdit extends OpMode
{

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Declare OpMode members.
    final ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightRear = null;
    //private DcMotor intake = null;
    //private DcMotor shooter = null;
    //private Servo intaketrigger;
    //Servo trigger;
    //Servo elevatorleft;
    //Servo elevatorright;
    //Servo slope;
    //Servo triggerroller;
    //DcMotor leftencoder;
   // DcMotorEx motor;

    //pid变量
    double propotionAngle=0,integralAngle=0,differentiationAngle=0;
    double lastAngleErr;
    //pid参数
    double Kp=0.45,Ki=0.01,Kd=0.04;

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
//        intake        = hardwareMap.get(DcMotor.class, "intake");
//        shooter       = hardwareMap.get(DcMotor.class, "shooter");
//        intaketrigger = hardwareMap.get(Servo.class, "intaketrigger");
//        trigger       = hardwareMap.get(Servo.class, "trigger");
//        elevatorleft  = hardwareMap.get(Servo.class, "elevatorleft");
//        elevatorright = hardwareMap.get(Servo.class, "elevatorright");
//        slope         = hardwareMap.get(Servo.class, "slope");
//        triggerroller = hardwareMap.get(Servo.class, "triggerroller");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
//        intake.setDirection(DcMotor.Direction.REVERSE);
//        shooter.setDirection(DcMotor.Direction.FORWARD);
//        triggerroller.setDirection(Servo.Direction.FORWARD); // set triggerroller direction
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        float x;
        float y;
        float lt;
        float rt;
        float intakePower;

        double speedModify;
        double targetAngle = 0;

        //double DCPower = 1;
        boolean triggerPosition;
        final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int    CYCLE_MS    =   50;     // period of each cycle
        final double MAX_POS     =  1.0;     // Maximum rotational position
        final double MIN_POS     =  0.0;     // Minimum rotational position



        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        // double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        y            = -gamepad1.left_stick_y ;
        x            = gamepad1.right_stick_x ;
        lt           = gamepad1.left_trigger;
        rt           = gamepad1.right_trigger;
        intakePower  =-gamepad2.left_stick_y;

        //获取修正速度
        speedModify=gyroMofifyPIF(targetAngle);
        LeftFront.setPower(((y+x)+(rt-lt)) + speedModify);
        LeftRear.setPower(((y-x)+(rt-lt)) + speedModify);
        RightFront.setPower(((y-x)+(lt-rt)) + speedModify);
        RightRear.setPower(((y+x)+(lt-rt)) + speedModify);
//        intake.setPower(intakePower);
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

        //telemetry.addData("encoder value:",motor.getCurrentPosition());
        //telemetry.addData("IntakePower", "left (%.2f), right (%.2f)", intakePower, shooterPower);
        //telemetry.addData("ShooterPower", "left (%.2f)", shooterPower);

        telemetry.update();

    }

    public double gyroMofifyPIF(double targetAngle) {
        Orientation angle;
        angle=imu.getAngularOrientation();
        //计算比例误差
        propotionAngle=targetAngle-angle.firstAngle;
        //微分误差
        differentiationAngle=propotionAngle-lastAngleErr;
        //积分误差
        integralAngle=integralAngle+propotionAngle;

        lastAngleErr=propotionAngle;
        //输出修正速度
        return (Kp*propotionAngle+Ki*integralAngle+Kd*differentiationAngle)/15;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
