package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
import org.firstinspires.ftc.teamcode.subsys.intake;

@TeleOp(name = "Tele Red", group = "TeleOp")
public class TeleOpRed extends OpMode {

    private Shooter shooter;
    private intake intakee;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private IMU imu;

    // Ganho Proporcional do giro automático (Ajusta se o giro estiver muito brusco ou fraco)
    private static final double KP_ALIGN = 0.03;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define folga no analógico para matar o drift de comandos indesejados


        imu = hardwareMap.get(IMU.class, "imu");

        shooter = new Shooter(hardwareMap);
        intakee = new intake((hardwareMap));

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Aperta 'A' para zerar o Yaw");
        telemetry.addLine("Segura 'Y' para girar e alinhar em 0°");
        telemetry.addLine("Segura Left Bumper para Robot-Relative");

        /*if (gamepad1.a) {
            imu.resetYaw();
        }*/

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if ((gamepad2.right_trigger) > 0.8) {
            shooter.shoot(1.0);
        }
        else {
            shooter.stop();
        }

        if ((gamepad2.left_trigger) > 0.8)  {
            intakee.enter(1.0);
        } else if ((gamepad2.left_bumper)) {
            intakee.out(1.0);

        }


        if (gamepad1.left_bumper) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }

        telemetry.addData("Yaw Atual (Graus)", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double currentYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // SE SEGURAR 'Y': O IMU assume o cálculo da rotação para mirar nos 0°
        if (gamepad1.y) {
            double targetAngleRad = 0.0; // Ponto 0° em radianos
            double headingError = AngleUnit.normalizeRadians(targetAngleRad - currentYawRad);

            // Transforma o erro em graus e aplica o ganho proporcional Kp
            rotate = Math.toDegrees(headingError) * KP_ALIGN;
        }

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - currentYawRad);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}