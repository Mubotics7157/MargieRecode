// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
    // Queue to read inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOTalonFXReal(SwerveModuleConstants constants) {
        super(constants);

        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Apply coupling compensation to main drive position
        // When azimuth rotates, the drive encoder sees phantom movement
        double couplingRatio = constants.CouplingGearRatio;
        double driveGearRatio = constants.DriveMotorGearRatio;
        double turnRotations = inputs.turnAbsolutePosition.getRotations();
        double couplingCorrectionRad = Units.rotationsToRadians(turnRotations * couplingRatio / driveGearRatio);
        inputs.drivePositionRad -= couplingCorrectionRad;

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        // Get raw positions from queues
        double[] rawDrivePositions = drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
        double[] rawTurnPositions = turnPositionQueue.stream().mapToDouble(Double::doubleValue).toArray();

        // Apply coupling compensation to odometry samples
        inputs.odometryDrivePositionsRad = new double[rawDrivePositions.length];
        for (int i = 0; i < rawDrivePositions.length; i++) {
            double couplingCorrection = rawTurnPositions[i] * couplingRatio / driveGearRatio;
            inputs.odometryDrivePositionsRad[i] = Units.rotationsToRadians(rawDrivePositions[i] - couplingCorrection);
        }

        inputs.odometryTurnPositions = new Rotation2d[rawTurnPositions.length];
        for (int i = 0; i < rawTurnPositions.length; i++) {
            inputs.odometryTurnPositions[i] = Rotation2d.fromRotations(rawTurnPositions[i]);
        }

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }
}
