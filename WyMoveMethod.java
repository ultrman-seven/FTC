package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class WyMoveMethod {
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;

    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor mid = null;

    private BNO055IMU imu = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    public void init(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2) {
        hardwareMap = h;
        telemetry = t;
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void motorInit(String lf, String lr, String rf, String rr) {
        leftFront = hardwareMap.get(DcMotor.class, lf);
        leftRear = hardwareMap.get(DcMotor.class, lr);
        rightFront = hardwareMap.get(DcMotor.class, rf);
        rightRear = hardwareMap.get(DcMotor.class, rr);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void imuInit(String imu_name) {
        imu = hardwareMap.get(BNO055IMU.class, imu_name);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.accelerationIntegrationAlgorithm = new WyAccInt();
        //延时0.5秒，以确保imu正常工作
        try {
            Thread.sleep(1000);//单位：毫秒
        } catch (Exception e) {
        }
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(),new Velocity(),1000);
    }

    public void extraEncoderInit(DcMotor l, DcMotor r, DcMotor m) {
        left = l;
        right = r;
        mid = m;

        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void showIMU() {
        Position p =imu.getParameters().accelerationIntegrationAlgorithm.getPosition();

        telemetry.addData("xyz","%f %f %f",p.x,p.y,p.z);
        Acceleration a = imu.getLinearAcceleration();//imu.getAcceleration();

        telemetry.addData("acc","x:%f,y:%f,z:%f",a.xAccel,a.yAccel,a.zAccel);
//        Orientation angle;
//        angle = imu.getAngularOrientation();
//        telemetry.addData("第1个角", angle.firstAngle);
//        telemetry.addData("第2个角", angle.secondAngle);
//        telemetry.addData("第3个角", angle.thirdAngle);
    }

    private double targetAngle = 0;
    private boolean lbFlag = false, rbFlag = false;//左右bumper是否按下标志，以确保每次按下之后只调整一次角度
    public void basicMoveThroughGamePad() {
        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        double speedModify = gyroModifyPID();
        leftFront.setPower(y + x - speedModify);
        leftRear.setPower(y - x - speedModify);
        rightFront.setPower(y - x + speedModify);
        rightRear.setPower(y + x + speedModify);

        final double angleIncreaseCoefficient = 3.5;
        targetAngle += angleIncreaseCoefficient * gamepad1.left_trigger;
        targetAngle -= angleIncreaseCoefficient * gamepad1.right_trigger;
        targetAngle -= angleIncreaseCoefficient * gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            if (!lbFlag) {
                lbFlag = true;
                targetAngle += 5;
            }
        } else
            lbFlag = false;
        if (gamepad1.right_bumper) {
            if (!rbFlag) {
                rbFlag = true;
                targetAngle -= 5;
            }
        } else
            rbFlag = false;
        if (gamepad1.b)
            targetAngle = 0;
        if (targetAngle > 180)
            targetAngle -= 360;
        if (targetAngle < -180)
            targetAngle += 360;
    }

    public void showEncoders() {
        Integer lf, lr, rf, rr, l, m, r;
        lf = leftFront.getCurrentPosition();
        rf = rightFront.getCurrentPosition();
        rr = rightRear.getCurrentPosition();
        lr = leftRear.getCurrentPosition();
        l = left.getCurrentPosition();
        m = mid.getCurrentPosition();
        r = -right.getCurrentPosition();

        telemetry.addData("左前轮", lf);
        telemetry.addData("左后轮", lr);
        telemetry.addData("右前轮", rf);
        telemetry.addData("右后轮", rr);
        telemetry.addData("左编码器", l);
        telemetry.addData("中间编码器", m);
        telemetry.addData("右编码器", r);
    }

    public double position_x, position_y;
    private void positionUpdate() {
    }

    public void turnToAngle(float angle) {
        targetAngle = angle;
    }
    public void turnToAngleImmediately(float angle) {
        targetAngle = angle;
        waitTillAngle();
    }
    public void turnAngle(double angle) {
        targetAngle += angle;
        if (targetAngle > 180)
            targetAngle -= 360;
        if (targetAngle < -180)
            targetAngle += 360;
    }
    public void turnAngleImmediately(double angle) {
        turnAngle(angle);
        waitTillAngle();
    }
    private void waitTillAngle() {
        double speed = gyroModifyPID();
        while (Math.abs(speed) > 0.05) {
            leftRear.setPower(-speed);
            leftFront.setPower(-speed);
            rightRear.setPower(speed);
            rightFront.setPower(speed);
            speed = gyroModifyPID();
        }
        speed = 0;
        leftRear.setPower(-speed);
        leftFront.setPower(-speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);
    }


    private double integralAngle = 0;//pid变量
    private double lastAngleErr = 0;
    private double gyroModifyPID() {
        final double Kp = 0.45, Ki = 0.0, Kd = 0.04;//pid参数
        //final double Kp = 0.45, Ki = -0.2, Kd = 0.05;//pid参数
        double proportionAngle, differentiationAngle;
        Orientation angle;
        angle = imu.getAngularOrientation();
        proportionAngle = targetAngle - angle.firstAngle;//计算角度误差，作为比例误差
        if (proportionAngle > 200)
            proportionAngle -= 360;
        if (proportionAngle < -200)
            proportionAngle += 360;
        differentiationAngle = proportionAngle - lastAngleErr;//微分
        integralAngle = integralAngle + proportionAngle;//积分
        lastAngleErr = proportionAngle;
        //telemetry.addData("mjj", targetAngle);
        return (Kp * proportionAngle + Ki * integralAngle + Kd * differentiationAngle) / 15;
    }
}