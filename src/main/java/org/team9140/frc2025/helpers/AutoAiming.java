package org.team9140.frc2025.helpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import org.team9140.frc2025.Constants;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

public class AutoAiming {
    // DO NOT CHANGE THE ORDER OF THESE UNLESS YOU KNOW WHAT YOU'RE DOING
    public enum ReefFaces {
        BlueFarMiddle(Constants.FieldItemPoses.REEF_BLUE, Sides.FarMiddle, true),
        BlueFarLeft(Constants.FieldItemPoses.REEF_BLUE, Sides.FarLeft, true),
        BlueCloseLeft(Constants.FieldItemPoses.REEF_BLUE, Sides.CloseLeft, false),
        BlueCloseMiddle(Constants.FieldItemPoses.REEF_BLUE, Sides.CloseMiddle, false),
        BlueCloseRight(Constants.FieldItemPoses.REEF_BLUE, Sides.CloseRight, false),
        BlueFarRight(Constants.FieldItemPoses.REEF_BLUE, Sides.FarRight, true),
        RedCloseMiddle(Constants.FieldItemPoses.REEF_RED, Sides.CloseMiddle, false),
        RedCloseRight(Constants.FieldItemPoses.REEF_RED, Sides.CloseRight, false),
        RedFarRight(Constants.FieldItemPoses.REEF_RED, Sides.FarRight, true),
        RedFarMiddle(Constants.FieldItemPoses.REEF_RED, Sides.FarMiddle, true),
        RedFarLeft(Constants.FieldItemPoses.REEF_RED, Sides.FarLeft, true),
        RedCloseLeft(Constants.FieldItemPoses.REEF_RED, Sides.CloseLeft, false);

        enum Sides {
            CloseMiddle(Radians.of(Math.PI)),
            CloseRight(Radians.of(-2*Math.PI / 3)),
            CloseLeft(Radians.of(2*Math.PI / 3)),
            FarMiddle(Radians.of(0)),
            FarRight(Radians.of(-Math.PI / 3)),
            FarLeft(Radians.of(Math.PI / 3));

            private Transform2d offset;
            private Rotation2d direction;

            Sides(Angle angleToCenter) {
                this.direction = new Rotation2d(angleToCenter.plus(Radians.of(Math.PI)));
                this.offset = new Transform2d(new Translation2d(Constants.REEF_RADIUS.in(Meters), new Rotation2d(angleToCenter)), this.direction);
            }

            public Transform2d getOffset() {
                return this.offset;
            }

            public Rotation2d getDirection() {
                return this.direction;
            }
        }

        private Pose2d pose;
        private Rotation2d direction;
        private boolean isFar;

        ReefFaces(Pose2d center, Sides face, boolean isFar) {
            this.isFar = isFar;

            if (center.equals(Constants.FieldItemPoses.REEF_RED)) {
                this.pose = center.plus(
                        face.getOffset().plus(new Transform2d(0, 0, new Rotation2d(Radians.of(Math.PI))))
                );
                this.direction = face.getDirection().plus(new Rotation2d(Math.PI));
            } else {
                this.pose = center.plus(face.getOffset());
                this.direction = face.getDirection();
            }
        }

        public Pose2d getPose() {
            return this.pose;
        }

        public boolean isFar() {
            return this.isFar;
        }

        public Rotation2d getDirection() {
            return this.direction;
        }
    }

    public static ReefFaces getClosestFace(Translation2d pose) {
        // Before Middle
        if (pose.getX() < 17.548225 / 2) {
            double angle = MathUtil.inputModulus(pose.minus(Constants.FieldItemPoses.REEF_BLUE.getTranslation()).getAngle().getDegrees() + 30, 0, 360);
            ReefFaces[] branches = ReefFaces.values();

            return branches[(int) (angle / 60)];
        } else {
            double angle = MathUtil.inputModulus(pose.minus(Constants.FieldItemPoses.REEF_RED.getTranslation()).getAngle().getDegrees() + 30, 0, 360);
            ReefFaces[] branches = ReefFaces.values();

            return branches[6 + (int) (angle / 60)];
        }
    }

    // return the tag id# of the apriltag on the closest reef side to given pose
    public static int reefTagFromPose(Pose2d pose) {
        return 0;
    }

    public static Optional<Pose2d> snapPose(int tagID, int level, boolean left) {
        // find pose of tag from id#
        // need it as a pose2d with the Z removed
        // Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).orElse(null);

        // placeholder 
        Pose2d tagPose = new Pose2d();

        Translation2d offset = new Translation2d();

        if (left) {
            if (level == 1) {
                offset = Constants.AutoAlign.leftBranchOffset_L1;
            } else if (level == 2) {
                offset = Constants.AutoAlign.leftBranchOffset_L2;
            } else if  (level == 3) {
                // bla bla finish this if statement tree
            }
        } else {

        }

        // rotate the offset around based on which way the tag points
        offset = offset.rotateBy(tagPose.getRotation());

        // add rotated offset to tag pose
        return Optional.of(tagPose.plus(new Transform2d(offset, new Rotation2d())));
    }
}
