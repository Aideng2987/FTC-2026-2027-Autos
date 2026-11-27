package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveLocalizer implements Localizer {

    // MOTOR & IMU NAMES (must match your Robot Configuration on the Driver Station)
    private static final String FRONT_LEFT_NAME = "topLeftWheel";
    private static final String BACK_LEFT_NAME = "bottomLeftWheel";
    private static final String FRONT_RIGHT_NAME = "topRightWheel";
    private static final String BACK_RIGHT_NAME = "bottomRightWheel";
    private static final String IMU_NAME = "imu";

    // definetly need to change this cus robot isn't done
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // WHEEL CONSTANTS  for goBILDA 312 RPM 537.7 ticks/rev + 96mm wheels
    // Inches per tick = (pi * 3.7795) / 537.7 = 0.022083
    public static double FORWARD_INCHES_PER_TICK = 0.022083;
    // Mecanum wheels lose ~12% efficiency sideways; tweak this after pushing 48" sideways
    public static double STRAFE_INCHES_PER_TICK = 0.0195;

    // Flip to -1 if pushing the robot forward makes that wheel's ticks go negative
    // Also errors could be here
    public static int FL_SIGN = 1;
    public static int BL_SIGN = 1;
    public static int FR_SIGN = 1;
    public static int BR_SIGN = 1;

    private final DcMotor fl, bl, fr, br;
    private final IMU imu;

    private int lastFl, lastBl, lastFr, lastBr;
    private double lastRawHeading = 0.0;
    private double headingOffset = 0.0;

    private Pose currentPose = Pose.zero();
    private Velocity currentVelocity = Velocity.zero();
    private long lastUpdateNanos = 0L;

    public MecanumDriveLocalizer(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        bl = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        fr = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        br = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        resetEncoders();
        lastRawHeading = getRawImuHeading();
        lastUpdateNanos = System.nanoTime();
    }

    @Override
    public void setPose(Pose pose) {
        currentPose = pose;
        lastRawHeading = getRawImuHeading();
        headingOffset = AngleUnit.normalizeRadians(pose.heading() - lastRawHeading);
        resetEncoders();
        currentVelocity = Velocity.zero();
        lastUpdateNanos = System.nanoTime();
    }

    @Override
    public MotionState state() {
        return MotionState.ofVelocity(currentPose, currentVelocity);
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastUpdateNanos) / 1e9;
        lastUpdateNanos = now;
        int currentFrontLeft = fl.getCurrentPosition() * FL_SIGN;
        int currentBackleft = bl.getCurrentPosition() * BL_SIGN;
        int currentFrontright = fr.getCurrentPosition() * FR_SIGN;
        int currentBackright = br.getCurrentPosition() * BR_SIGN;

        int dFl = currentFrontLeft - lastFl;
        int dBl = currentBackleft - lastBl;
        int dFr = currentFrontright - lastFr;
        int dBr = currentBackright - lastBr;

        lastFl = currentFrontLeft;
        lastBl = currentBackleft;
        lastFr = currentFrontright;
        lastBr = currentBackright;

        // Robot-frame movement
        double dForward = ((dFl + dBl + dFr + dBr) / 4.0) * FORWARD_INCHES_PER_TICK;
        double dStrafe = ((-dFl + dBl + dFr - dBr) / 4.0) * STRAFE_INCHES_PER_TICK;

        // Heading from IMU
        double rawHeading = getRawImuHeading();
        double dHeading = AngleUnit.normalizeRadians(rawHeading - lastRawHeading);
        lastRawHeading = rawHeading;

        double newHeading = AngleUnit.normalizeRadians(rawHeading + headingOffset);
        double avgHeading = AngleUnit.normalizeRadians(currentPose.heading() + (dHeading / 2.0));

        // rotate robot movement into field coordinates
        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);
        double dX = dForward * cos - dStrafe * sin;
        double dY = dForward * sin + dStrafe * cos;

        currentPose = new Pose(currentPose.x() + dX, currentPose.y() + dY, newHeading);

        if (dt > 1e-6) {
            currentVelocity = new Velocity(dX / dt, dY / dt, dHeading / dt);
        }
    }

    @Override
    public void reset() {
        setPose(Pose.zero());
    }

    private void resetEncoders() {
        lastFl = fl.getCurrentPosition() * FL_SIGN;
        lastBl = bl.getCurrentPosition() * BL_SIGN;
        lastFr = fr.getCurrentPosition() * FR_SIGN;
        lastBr = br.getCurrentPosition() * BR_SIGN;
    }

    private double getRawImuHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}