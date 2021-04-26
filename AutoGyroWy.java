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

//import android.graphics.RectF;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class AutoGyroWy extends LinearOpMode {

    // The IMU sensor object
    BNO055IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    final int readingPerRound = 4096;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private DcMotor LeftFront = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightRear = null;

    double propotionAngle=0, integralAngle=0, differentiationAngle=0;
    double propotionEncoder=0, integralEncoder=0, differentiationEncoder=0;
    double lastAngleErr;
    double lastEncoderErr;
    double Kp=0.45,Ki=0.01,Kd=0.04;
    double Kp_encoder=0.45, Ki_encoder=0.01, Kd_encoder=0.04;
    double currentAngle = 0;

    @Override
    public void runOpMode() {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */

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

        //
        LeftFront     = hardwareMap.get(DcMotor.class, "leftfront");
        LeftRear      = hardwareMap.get(DcMotor.class, "leftrear");
        RightFront    = hardwareMap.get(DcMotor.class, "rightfront");
        RightRear     = hardwareMap.get(DcMotor.class, "rightrear");

        //
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();


        // make sure the gyro is calibrated before continuing
        while (!isStopRequested())  {
            sleep(50);
            idle();
        }

        imu.initialize(parameters);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        encoderReset(RightFront);
        encoderReset(LeftFront);
        encoderReset(RightRear);
        encoderReset(LeftRear);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
        }

        moveForward(10*readingPerRound);
        turnAngle(10);
        moveTrans(5*readingPerRound);
        moveForward(5*readingPerRound);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * **************************gyroModifyPID***********************************
     * 功能描述：根据目标角，返回速度修正。
     *          修正后的效果为：车头指向角度targetAngle。
     * 使用方法：1. 将右边电机速度加上修正值，左边减去修正值。
     *          2. 调整targetAngle可以实现转向。
     * ***************************************************************************
     * */
    public double gyroModifyPID(double targetAngle) {
        Orientation angle;
        angle=imu.getAngularOrientation();

        propotionAngle=targetAngle-angle.firstAngle;
        differentiationAngle=propotionAngle-lastAngleErr;
        integralAngle += propotionAngle;
        lastAngleErr = propotionAngle;

        return (Kp*propotionAngle+Ki*integralAngle+Kd*differentiationAngle)/15;
    }

    public double encoderModifyPID(int left, int right){
        double modify;
        propotionEncoder=(left-right)/(left+right);
        differentiationEncoder = propotionEncoder - lastEncoderErr;
        integralEncoder += propotionEncoder;
        lastEncoderErr = propotionEncoder;
        modify = Kp_encoder*propotionEncoder + Ki_encoder*integralEncoder + Kd_encoder*differentiationEncoder;
        return modify;
    }

    public void moveForward(int targetPosition){
        int leftEncoder = 0, rightEncoder = 0;
        double speedModify = 0;
        final double speed = 0.2;
        forwardPower(speed,speedModify);
        while ( ((leftEncoder+rightEncoder)/2) < targetPosition)
        {
            leftEncoder = -LeftFront.getCurrentPosition()-1;
            rightEncoder = -RightFront.getCurrentPosition()-1;
            //speedModify = encoderModifyPID(leftEncoder, rightEncoder);//编码器pid
            speedModify =0;// gyroModifyPID(targetAngle);//陀螺仪pid
            forwardPower(speed,speedModify);
        }
        forwardPower(0,0);
        encoderReset(RightFront);
        encoderReset(LeftFront);
    }

    public void moveTrans(int targetPosition){
        int midEncoder = 0;
        double speedModify = 0;
        final double speed = 0.8;
        transPower(speed,speedModify);
        while (midEncoder < targetPosition){
            midEncoder = RightRear.getCurrentPosition();
            speedModify = gyroModifyPID(currentAngle);
            transPower(speed,speedModify);
        }
        transPower(0,0);
        encoderReset(RightRear);
    }

    public void turnAngle(double angle){
        double speedModify;
        currentAngle = angle;
        speedModify = gyroModifyPID(angle);
        //while ( (speedModify > 0.1) || (speedModify < -0.1) ){
        while (imu.getAngularOrientation().firstAngle != angle){
            forwardPower(0,speedModify);
        }
    }

    public void forwardPower(double speed, double modify){
        RightFront.setPower(speed + modify);
        RightRear.setPower(speed + modify);
        LeftFront.setPower(speed - modify);
        LeftRear.setPower(speed - modify);
    }

    public void transPower(double speed, double modify){
        RightFront.setPower(-speed + modify);
        RightRear.setPower(speed + modify);
        LeftFront.setPower(speed - modify);
        LeftRear.setPower(-speed - modify);
    }

    public void encoderReset(DcMotor mo){
        mo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
