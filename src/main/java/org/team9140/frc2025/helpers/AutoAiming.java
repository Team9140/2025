package org.team9140.frc2025.helpers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Distance;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.ElevatorSetbacks;
import org.team9140.frc2025.Constants.FieldItemPoses;
import org.team9140.frc2025.Constants.AutoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class AutoAiming {
    // DO NOT CHANGE THE ORDER OF THESE UNLESS YOU KNOW WHAT YOU'RE DOING
    public enum ReefFaces {
        BlueFarMiddle(FieldItemPoses.REEF_BLUE, Sides.Right, true, 21, Constants.Elevator.L2_ALGAE_height),
        BlueFarLeft(FieldItemPoses.REEF_BLUE, Sides.TopRight, true, 20, Constants.Elevator.L3_ALGAE_height),
        BlueCloseLeft(FieldItemPoses.REEF_BLUE, Sides.TopLeft, false, 19, Constants.Elevator.L2_ALGAE_height),
        BlueCloseMiddle(FieldItemPoses.REEF_BLUE, Sides.Left, false, 18, Constants.Elevator.L3_ALGAE_height),
        BlueCloseRight(FieldItemPoses.REEF_BLUE, Sides.BottomLeft, false, 17, Constants.Elevator.L2_ALGAE_height),
        BlueFarRight(FieldItemPoses.REEF_BLUE, Sides.BottomRight, true, 22, Constants.Elevator.L3_ALGAE_height),
        RedCloseMiddle(FieldItemPoses.REEF_RED, Sides.Right, false, 7, Constants.Elevator.L2_ALGAE_height),
        RedCloseRight(FieldItemPoses.REEF_RED, Sides.TopRight, false, 8, Constants.Elevator.L3_ALGAE_height),
        RedFarRight(FieldItemPoses.REEF_RED, Sides.TopLeft, true, 9, Constants.Elevator.L2_ALGAE_height),
        RedFarMiddle(FieldItemPoses.REEF_RED, Sides.Left, true, 10, Constants.Elevator.L3_ALGAE_height),
        RedFarLeft(FieldItemPoses.REEF_RED, Sides.BottomLeft, true, 11, Constants.Elevator.L2_ALGAE_height),
        RedCloseLeft(FieldItemPoses.REEF_RED, Sides.BottomRight, false, 6, Constants.Elevator.L3_ALGAE_height);

        // Birds eye view
        private enum Sides {
            Left(Radians.of(Math.PI)),
            BottomLeft(Radians.of(-2 * Math.PI / 3)),
            TopLeft(Radians.of(2 * Math.PI / 3)),
            Right(Radians.of(0)),
            BottomRight(Radians.of(-Math.PI / 3)),
            TopRight(Radians.of(Math.PI / 3));

            private final Transform2d offset;
            private final Rotation2d direction;

            Sides(Angle angleToCenter) {
                this.direction = new Rotation2d(angleToCenter.plus(Radians.of(Math.PI)));
                this.offset = new Transform2d(
                        new Translation2d(AutoAlign.REEF_RADIUS.in(Meters), new Rotation2d(angleToCenter)),
                        this.direction);
            }

            public Transform2d getOffset() {
                return this.offset;
            }

            public Rotation2d getDirection() {
                return this.direction;
            }
        }

        private final Pose2d pose;
        private final Rotation2d direction;
        private final boolean isFar;
        private final int tagId;
        private final Distance algaeElevatorHeight;

        ReefFaces(Pose2d center, Sides face, boolean isFar, int tagId, Distance algaeElevatorHeight) {
            this.isFar = isFar;
            this.tagId = tagId;

            this.pose = center.plus(face.getOffset());
            this.direction = face.getDirection();

            this.algaeElevatorHeight = algaeElevatorHeight;
        }

        public Pose2d getCenter() {
            return this.pose.plus(this.algaeElevatorHeight.equals(Constants.Elevator.L2_ALGAE_height) ? Constants.ElevatorSetbacks.ALGAE_L2.getSetbackinator() : ElevatorSetbacks.ALGAE_L3.getSetbackinator());
        }

        public Pose2d getLeft(ElevatorSetbacks height) {
            return this.pose
                    .plus(AutoAlign.HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER.times(this.isFar ? -1 : 1))
                    .plus(height.getSetbackinator());
        }

        public Pose2d getRight(ElevatorSetbacks height) {
            return this.pose
                    .plus(AutoAlign.HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER.times(this.isFar ? 1 : -1))
                    .plus(height.getSetbackinator());
        }

        public Distance getAlgaeElevatorHeight() {
            return this.algaeElevatorHeight;
        }

        public Rotation2d getDirection() {
            return this.direction;
        }

        public int getTagId() {
            return this.tagId;
        }
    }

    public static ReefFaces getClosestFace(Translation2d pose) {
        // Before Middle
        if (pose.getX() < 17.548225 / 2) {
            double angle = MathUtil.inputModulus(
                    pose.minus(FieldItemPoses.REEF_BLUE.getTranslation()).getAngle().getDegrees() + 30, 0,
                    360);
            ReefFaces[] branches = ReefFaces.values();

            return branches[(int) (angle / 60)];
        } else {
            double angle = MathUtil.inputModulus(
                    pose.minus(FieldItemPoses.REEF_RED.getTranslation()).getAngle().getDegrees() + 30, 0,
                    360);
            ReefFaces[] branches = ReefFaces.values();

            return branches[6 + (int) (angle / 60)];
        }
    }
}
