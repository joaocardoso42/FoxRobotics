package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private IMU imu;

    // --- CONFIGURAÇÕES DO TEU ROBÔ ---
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_CM = 9.6;
    // CORRIGIDO: Adicionado o * que tinha sumido
    private static final double COUNTS_PER_CM = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_CM);

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBrake();

        // INICIALIZAÇÃO DO IMU (Control Hub)
        imu = hardwareMap.get(IMU.class, "imu");

        // Ajusta a orientação do Hub no robô (Ajusta LOGO_FACING e USB_FACING se o teu Hub estiver de pé/de lado!)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // --- MOVIMENTAÇÃO TELEOP / CAMPO RELATIVO ---
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    // --- AUTÔNOMO: FRENTE E TRÁS POR CM ---
    public void driveDistanceCM(double distanceCM, double power) {
        int moveTicks = (int) (distanceCM * COUNTS_PER_CM);

        resetEncoders();

        frontLeft.setTargetPosition(moveTicks);
        frontRight.setTargetPosition(moveTicks);
        backLeft.setTargetPosition(moveTicks);
        backRight.setTargetPosition(moveTicks);

        setRunToPosition();
        setMotorPowers(Math.abs(power));

        while (isBusy()) {
            // Espera chegar no destino
        }

        stop();
        setRunWithoutEncoder();
    }

    // --- AUTÔNOMO: STRAFE LATERAL POR CM ---
    public void strafeDistanceCM(double distanceCM, double power) {
        double STRAFE_COMPENSATION = 1.15;
        // CORRIGIDO: Adicionado o * que tinha sumido
        int moveTicks = (int) (distanceCM * COUNTS_PER_CM * STRAFE_COMPENSATION);

        resetEncoders();

        frontLeft.setTargetPosition(moveTicks);
        frontRight.setTargetPosition(-moveTicks);
        backLeft.setTargetPosition(-moveTicks);
        backRight.setTargetPosition(moveTicks);

        setRunToPosition();
        setMotorPowers(Math.abs(power));

        while (isBusy()) {
            // Espera chegar no destino
        }

        stop();
        setRunWithoutEncoder();
    }

    // --- AUTÔNOMO: GIRO POR GRAUS USANDO O IMU INTERNO ---
    public void turnDegrees(double targetDegrees, double powerLimit) {
        double KP = 0.025;
        double targetRadians = Math.toRadians(targetDegrees);

        stop();
        setZeroPowerBrake();

        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = AngleUnit.normalizeRadians(targetRadians - currentYaw);

        while (Math.abs(headingError) > 0.025) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingError = AngleUnit.normalizeRadians(targetRadians - currentYaw);

            double turnPower = headingError * KP;

            if (turnPower > powerLimit) turnPower = powerLimit;
            if (turnPower < -powerLimit) turnPower = -powerLimit;

            drive(0, 0, turnPower);
        }

        stop();
    }

    // --- MÉTODOS AUXILIARES ---
    public IMU getImu() {
        return imu;
    }

    public void stop() {
        setMotorPowers(0);
    }

    private void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void setMotorPowers(double power) {
        setPowers(power, power, power, power);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setRunWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    public void setZeroPowerBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}