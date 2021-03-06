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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class AutoGyroWy extends LinearOpMode {

    //-------------------------------???????????????????????????--------------------------------------------
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
    private TouchSensor touchBehind;
    private TouchSensor touchUpon;
    private DigitalChannel touchSensor0;
    private DigitalChannel touchSensor1;
    BNO055IMU imu;
    //--------------------------------????????????------------------------------------------------------

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AXx1c7T/////AAABmbPVD9EaVkFpuhQbYfj4x8CIvCZt0o7tF/ShBB0CReh7wngOy66mktwoIH/3qHhDGwSAEFgT20ZrmOgOShRWYb4jMrNv2M7617K/wMTA6M/WFKONhSS8T+6STFimWCGU6K9x4hvP+meU3i38E1+rCdif5h4tI6m9qL3J3wKG5Wb5CCTThSbbVNSbybtQCVj7lrthoXtVW3F1lye444fLhg7QrEx3QxZI+GCM355GnxuPAxHaFONrtnBKBsae0Nl20EN8MaMu72lM1uvo8kzonVyo1SMbgXBMFwvzhFJ2cnVXOaTW1sIIzsIqSJ8IcqE6qnmrV2O7dbaHspZh40PBaLp6Mu3MZgVjs2JI4e4sqNwV";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        defInit();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
        }


//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                // step through the list of recognitions and display boundary info.
//                int i = 0;
//                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                            recognition.getLeft(), recognition.getTop());
//                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                            recognition.getRight(), recognition.getBottom());
//                }
//                telemetry.update();
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }

        resetTrigger();
        String route1 ="CL," + "A,280," + "T,90," + "DOWN," + "OP," + "A,30.48,"
                + "UP," + "CL," + "T,0," + "B,243.84," + "OP," + "DOWN," + "CL,"
                + "UP," + "A,243.84," + "T,90," + "B,30.48," + "DOWN," + "OP,"
                + "T,0," + "B,280," + "CL," + "UP," + "A,200,";

        String route2 ="CL,A,50,S,3,I,2500,S,1,A,50,T,-90,DOWN,OP,UP";

        behave(route2);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void defInit(){
        //??????imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //??????0.5???????????????imu????????????
        sleepForMS(300);

        //????????????
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
        touchBehind = hardwareMap.get(TouchSensor.class,"touchBehind");
        touchUpon = hardwareMap.get(TouchSensor.class,"touchUpon");
        touchSensor0 = hardwareMap.get(DigitalChannel.class, "touchsensor0");
        touchSensor1 = hardwareMap.get(DigitalChannel.class, "touchsensor1");

        //????????????
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
        //----------------------------??????????????????---------------------------------------------------
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //------------------------------------------------------------------------------------------

        imu.initialize(parameters);
        //??????0.5???????????????imu????????????
        sleepForMS(300);
    }

    public void behave(String str){
        String[] option = str.split(",");
        int i;
        Double value;
        for(i = 0; i < option.length; i++){
            switch (option[i]) {
                case "A":
                    value = new Double(option[++i]);
                    moveForward(value.doubleValue());
                    break;
                case "B":
                    value = new Double(option[++i]);
                    moveForward(-value.doubleValue());
                    break;
                case "L":
                    value = new Double(option[++i]);
                    moveTrans(value.doubleValue());
                    break;
                case "R":
                    value = new Double(option[++i]);
                    moveTrans(-value.doubleValue());
                    break;
                case "T":
                    value = new Double(option[++i]);
                    turnAngle(value.doubleValue());
                    break;
                case "UP":
                    armOption(ArmState.UP);
                    break;
                case "DOWN":
                    armOption(ArmState.DOWN);
                    break;
                case "OP":
                    clampOption(ClampState.OPEN);
                    break;
                case "CL":
                    clampOption(ClampState.CLOSE);
                    break;
                case "S":
                    value = new Double(option[++i]);
                    shootForTimes(value.intValue());
                    break;
                case "I":
                    value = new Double(option[++i]);
                    intakeForTime(value.intValue());
                    break;
            }
        }
    }

    /**
     * *****************          gyroModifyPID           ************************
     *
     * ??????????????????????????????????????????????????????
     *          ??????????????????????????????????????????targetAngle???
     * ???????????????1. ???????????????????????????????????????????????????????????????
     *          2. ??????targetAngle?????????????????????
     * ***************************************************************************
     * */
    double targetAngle = 0;
    double proportionAngle = 0,integralAngle = 0,differentiationAngle = 0;//pid??????
    double lastAngleErr;
    public double gyroModifyPID(double target) {
        final double Kp = 0.45, Ki = 0.0, Kd = 0.04;//pid??????
        Orientation angle;
        angle =imu.getAngularOrientation();
        proportionAngle=target-angle.firstAngle;//???????????????????????????????????????
        differentiationAngle=proportionAngle-lastAngleErr;//??????
        integralAngle=integralAngle+proportionAngle;//??????
        lastAngleErr=proportionAngle;
        telemetry.addData("angle","%.3f",angle.firstAngle);
        return (Kp*proportionAngle+Ki*integralAngle+Kd*differentiationAngle) / 15;
    }

    /**
     * ****************          encoderModifyPID           **********************
     *
     * ????????????????????????????????????????????????????????????
     *          ???????????????????????????????????????
     * ???????????????1. ???????????????????????????????????????????????????????????????
     * ***************************************************************************
     * */
    double proportionEncoder=0, integralEncoder=0, differentiationEncoder=0;
    double lastEncoderErr;
    public double encoderModifyPID(int left, int right){
        final double Kp = 0.45, Ki = 0.01, Kd = 0.04;
        double modify;
        proportionEncoder=(left-right)/(left+right);
        differentiationEncoder = proportionEncoder - lastEncoderErr;
        integralEncoder += proportionEncoder;
        lastEncoderErr = proportionEncoder;
        modify = Kp*proportionEncoder + Ki*integralEncoder + Kd*differentiationEncoder;
        return modify;
    }

    /**
     * ******************         intakeForTime            ************************
     *
     * ????????????: ???????????????,?????????
     * ????????????: ????????????????????????,????????????
     * ****************************************************************************
     * */
    public void intakeForTime(int time){
        intake.setPower(1);
        sleepForMS(time);
        intake.setPower(0);
    }

    public  void sleepForMS(int time){
        try {
            Thread.sleep(time);//???????????????
        } catch (Exception e) {
        }
    }

    /**
     * ****************          shootForTimes          ****************************
     *
     * ????????????: ??????n???
     *          ????????????????????? -> ??????n??? -> ????????????
     * ????????????: ?????????????????????n
     * *****************************************************************************
     * */
    final double TRI_RUN = 0.9;
    final double TRI_STOP =0.5;
    public void shootForTimes(int times){
        elevatorOperation(elevatorState.UP);
        int shootCount = 0;
        trigger.setPosition(TRI_RUN);
        while (true)
            if (touchSensor0.getState()) {
                if(shootCount++ < times)
                    sleepForMS(300);
                else break;
            }
        trigger.setPosition(TRI_STOP);
        elevatorOperation(elevatorState.DOWN);
    }

    /**
     * *******************      resetTrigger          ****************************
     * ????????????: trigger??????
     * ????????????:????????????
     * ***************************************************************************
     * */
    public void resetTrigger(){
        elevatorOperation(elevatorState.UP);
        while (true) {
            if (touchSensor1.getState())
                trigger.setPosition(TRI_RUN);
            if (touchSensor0.getState()) {
                trigger.setPosition(TRI_STOP);
                elevatorOperation(elevatorState.DOWN);
                break;
            }
        }
    }

    /**
     * ****************           elevatorOperation             ******************
     * ????????????: ?????????????????????
     * ????????????: ???????????????????????????
     * ***************************************************************************
     * */
    enum elevatorState{
        UP, DOWN;
    }
    public void elevatorOperation(elevatorState state){
        final double MAX_POS_left     =  0.65;
        final double MAX_POS_right    =  0.33;
        final double MIN_POS_left     =  0.33;
        final double MIN_POS_right     =  0.6;
        if(state == elevatorState.UP){
            elevatorLeft.setPosition(MAX_POS_left);
            elevatorRight.setPosition(MAX_POS_right);
        }
        if (state == elevatorState.DOWN){
            elevatorLeft.setPosition(MIN_POS_left);
            elevatorRight.setPosition(MIN_POS_right);
        }
    }

    /**
     * ********************        armOption           ****************************
     * ????????????: ???????????????/??????
     * ****************************************************************************
     * */
    enum ArmState{
        UP, DOWN;
    }
    public void armOption(ArmState armFlag) {
        if (armFlag == ArmState.UP) {
            arm.setPower(0.8);
            while (!touchUpon.isPressed())
                ;
            arm.setPower(0);
        }
        if(armFlag == ArmState.DOWN){
            arm.setPower(-0.8);
            while (!touchBehind.isPressed())
                ;
            arm.setPower(0);
        }
    }

    /**
     * ************************        clampOption       *************************
     * ????????????: ??????????????????
     * ????????????: ?????????????????????
     * ***************************************************************************
     * */
    enum ClampState{
        CLOSE, OPEN;
    }
    public void clampOption(ClampState state) {
        final double CLOSE = 0.26;
        final double OPEN = 0.7;
        if (state == ClampState.OPEN)
            clamp.setPosition(OPEN);
        if(state == ClampState.CLOSE)
            clamp.setPosition(CLOSE);
    }

    /**
     * **********************     moveForward       *******************************
     *
     * */
    public void moveForward(double targetPosition) {
        final int perRound = 4096;
        final double circumference = 3.8 * Math.PI * 1.0249167759507718;
        final double encoder_per_cm = perRound / circumference;
        int leftEncoder = 0, rightEncoder = 0;
        double speedModify = gyroModifyPID(targetAngle);
        final double speed = 1;
        if (targetPosition > 0) {
            forwardPower(speed, speedModify);
            while (((leftEncoder + rightEncoder) / 2) < targetPosition * encoder_per_cm) {
                leftEncoder = -LeftFront.getCurrentPosition() - 1;
                rightEncoder = -RightFront.getCurrentPosition() - 1;
                //speedModify = encoderModifyPID(leftEncoder, rightEncoder);//?????????pid
                speedModify = gyroModifyPID(targetAngle);//?????????pid
                forwardPower(speed, speedModify);
            }
        } else {
            forwardPower(-speed, speedModify);
            while (((leftEncoder + rightEncoder) / 2) > targetPosition * encoder_per_cm) {
                leftEncoder = -LeftFront.getCurrentPosition() - 1;
                rightEncoder = -RightFront.getCurrentPosition() - 1;
                //speedModify = encoderModifyPID(leftEncoder, rightEncoder);//?????????pid
                speedModify = gyroModifyPID(targetAngle);//?????????pid
                forwardPower(-speed, speedModify);
            }
        }
        forwardPower(0, 0);
        encoderReset(RightFront);
        encoderReset(LeftFront);
    }

    public void moveTrans(double targetPosition) {
        final int perRound = 4096;
        final double circumference = 3.8 * Math.PI * 1.0249167759507718;
        final double encoder_per_cm = perRound / circumference;
        int midEncoder = 0;
        double speedModify = gyroModifyPID(targetAngle);
        final double speed = 1;
        if (targetPosition > 0) {
            transPower(speed, speedModify);
            while (midEncoder < targetPosition * encoder_per_cm) {
                midEncoder = RightRear.getCurrentPosition();
                speedModify = gyroModifyPID(targetAngle);
                transPower(speed, speedModify);
            }
        }
        else {
            transPower(-speed, speedModify);
            while (midEncoder > targetPosition * encoder_per_cm) {
                midEncoder = RightRear.getCurrentPosition();
                speedModify = gyroModifyPID(targetAngle);
                transPower(-speed, speedModify);
            }
        }
        transPower(0, 0);
        encoderReset(RightRear);
    }

    public void turnAngle(double angle){
        double speedModify;
        targetAngle = angle;
        speedModify = gyroModifyPID(angle);
        while ( Math.abs(speedModify) > 0.1 ){
            speedModify = gyroModifyPID(angle);
            forwardPower(0,speedModify);
        }
        forwardPower(0,0);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
