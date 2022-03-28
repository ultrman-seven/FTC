package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class WyAccInt implements BNO055IMU.AccelerationIntegrator {
    BNO055IMU.Parameters parameters;
    Acceleration acceleration;
    private Position position;
    private Velocity velocity;

    @Override public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity)
    {
        this.parameters = parameters;
        position = initialPosition;
        velocity = initialVelocity;
    }

    @Override public Position getPosition() {
        return this.position;
    }
    @Override public Velocity getVelocity() {
        return this.velocity;
    }
    @Override public Acceleration getAcceleration()
    {
        return this.acceleration == null ? new Acceleration() : this.acceleration;
    }

    @Override public void update(Acceleration linearAcceleration)
    {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0)
        {
            if (acceleration != null)
            {
                Acceleration accelPrev = acceleration;
                acceleration = linearAcceleration;
                double dt = (accelPrev.acquisitionTime-acceleration.acquisitionTime)*1e-9;
                position.x += dt*velocity.xVeloc;
                position.y += dt*velocity.yVeloc;
                position.z += dt*velocity.zVeloc;
                velocity.xVeloc += dt*acceleration.xAccel;
                velocity.yVeloc += dt*acceleration.yAccel;
                velocity.zVeloc += dt*acceleration.zAccel;
                if (parameters.loggingEnabled)
                {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime)*1e-9, acceleration);
                }
            }
            else
                acceleration = linearAcceleration;
        }
    }
}
