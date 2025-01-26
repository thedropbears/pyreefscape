import math
import time

import wpilib
import wpiutil.log
from magicbot import feedback, tunable
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTag
from wpimath import objectToRobotPose
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation3d,
)

from components.chassis import ChassisComponent
from utilities.game import APRILTAGS, apriltag_layout
from utilities.scalers import scale_value


class VisualLocalizer:
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    # Give bias to the best pose by multiplying this const to the alt dist
    BEST_POSE_BIAS = 1.2

    # Time since the last target sighting we allow before informing drivers
    TIMEOUT = 1.0  # s

    CAMERA_PITCH = -math.radians(20)

    # Calibration constants for servo and encoder
    # range between +/-135 deg
    SERVO_HALF_ANGLE = math.radians(135)

    CAMERA_FOV = math.radians(
        65
    )  # photon vision says 69.8, but we are being conservative

    add_to_estimator = tunable(True)
    should_log = tunable(True)

    last_pose_z = tunable(0.0, writeDefault=False)
    linear_vision_uncertainty = tunable(0.3)
    rotation_vision_uncertainty = tunable(0.08)
    reproj_error_threshold = 0.1

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        # Position of the camera relative to the center of the robot
        pos: Translation3d,
        # The camera rotation at its neutral position (ie centred).
        rot: Rotation3d,
        servo_id: int,
        servo_offset: Rotation2d,
        encoder_id: int,
        encoder_offset: Rotation2d,
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
        chassis: ChassisComponent,
    ) -> None:
        self.camera = PhotonCamera(name)
        self.encoder = wpilib.DutyCycleEncoder(encoder_id, math.tau, 0)
        self.encoder.setAssumedFrequency(975.6)
        self.encoder.setDutyCycleRange(1 / 1025, 1024 / 1025)
        # Offset of encoder in radians when facing forwards (the desired zero)
        # To find this value, manually point the camera forwards and record the encoder value
        # This has nothing to do with the servo - do it by hand!!
        self.encoder_offset = encoder_offset

        # To find the servo offset, command the servo to neutral in test mode and record the encoder value
        self.servo_offset = servo_offset

        self.servo = wpilib.Servo(servo_id)
        self.pos = pos
        self.robot_to_turret = Transform3d(pos, rot)
        self.last_timestamp = -1.0
        self.last_recieved_timestep = -1.0
        self.best_log = field.getObject(name + "_best_log")
        self.alt_log = field.getObject(name + "_alt_log")
        self.field_pos_obj = field.getObject(name + "_vision_pose")
        self.pose_log_entry = wpiutil.log.FloatArrayLogEntry(
            data_log, name + "_vision_pose"
        )

        self.chassis = chassis
        self.current_reproj = 0.0
        self.has_multitag = False

    @feedback
    def reproj(self) -> float:
        return self.current_reproj

    @feedback
    def using_multitag(self) -> bool:
        return self.has_multitag

    @feedback
    def raw_encoder_rotation(self) -> Rotation2d:
        # The encoder has been set up to return values in the interval [0, 2pi]
        return Rotation2d(self.encoder.get())

    @feedback
    def relative_bearing_to_closest_tag(self) -> Rotation2d:
        # initialise default variables
        # closest tag, distance, robot position
        relative_bearings: list[Rotation2d] = []

        for tag in self.visible_tags():
            robot_to_tag = tag.pose.toPose2d() - self.chassis.get_pose()
            relative_bearing = robot_to_tag.translation().angle()
            relative_bearings.append(relative_bearing)
        relative_bearings.sort(key=Rotation2d.radians)
        for offset in range(len(relative_bearings) - 1, -1, -1):
            bearing_pairs = zip(relative_bearings, relative_bearings[offset:-1])
            for pair in bearing_pairs:
                if abs((pair[0] - pair[1]).radians()) < self.CAMERA_FOV:
                    return (pair[0] + pair[1]) * 0.5

        return Rotation2d(0.0)

    def visible_tags(self) -> list[AprilTag]:
        tags_in_view = []

        robot_pose = self.chassis.get_pose()

        for tag in APRILTAGS:
            robot_to_tag = tag.pose.toPose2d() - robot_pose
            relative_bearing = robot_to_tag.translation().angle()
            relative_facing = tag.pose.toPose2d().rotation() - robot_pose.rotation()
            if (
                abs(relative_bearing.radians()) <= self.SERVO_HALF_ANGLE
                and abs(relative_facing.degrees()) > 90
            ):
                tags_in_view.append(tag)

        return tags_in_view

    @feedback
    def visible_tag_ids(self) -> list[int]:
        return [tag.ID for tag in self.visible_tags()]

    @feedback
    def desired_turret_rotation(self) -> Rotation2d:
        # Read encoder angle and account for offset
        return self.relative_bearing_to_closest_tag() - self.chassis.get_rotation()

    def turret_to_servo(self, turret: Rotation2d) -> Rotation2d:
        return turret - (self.servo_offset - self.encoder_offset)

    @property
    def turret_rotation(self) -> Rotation2d:
        return self.raw_encoder_rotation() - self.encoder_offset

    @property
    def robot_to_camera(self) -> Transform3d:
        robot_to_turret_rotation = self.robot_to_turret.rotation()
        return Transform3d(
            self.robot_to_turret.translation(),
            Rotation3d(
                robot_to_turret_rotation.x,
                robot_to_turret_rotation.y,
                robot_to_turret_rotation.z + self.turret_rotation.radians(),
            ),
        )

    def zero_servo_(self) -> None:
        # ONLY CALL THIS IN TEST MODE!
        # This is used to put the servo in a neutral position to record the encoder value at that point
        self.servo.set(0.5)

    def execute(self) -> None:
        self.servo.set(
            scale_value(
                self.turret_to_servo(self.desired_turret_rotation()).radians(),
                -self.SERVO_HALF_ANGLE,
                self.SERVO_HALF_ANGLE,
                0.0,
                1.0,
            )
        )

        camera_to_robot = self.robot_to_camera.inverse()

        all_results = self.camera.getAllUnreadResults()
        for results in all_results:
            # if results didn't see any targets
            if not results.getTargets():
                return

            # if we have already processed these results
            timestamp = results.getTimestampSeconds()

            if timestamp == self.last_timestamp:
                return
            self.last_recieved_timestep = time.monotonic()
            self.last_timestamp = timestamp

            if results.multitagResult:
                self.has_multitag = True
                p = results.multitagResult.estimatedPose
                pose = (Pose3d() + p.best + camera_to_robot).toPose2d()
                reprojectionErr = p.bestReprojErr
                self.current_reproj = reprojectionErr

                self.field_pos_obj.setPose(pose)

                if (
                    self.add_to_estimator
                    and self.current_reproj < self.reproj_error_threshold
                ):
                    self.chassis.estimator.addVisionMeasurement(
                        pose,
                        timestamp,
                        (
                            self.linear_vision_uncertainty,
                            self.linear_vision_uncertainty,
                            self.rotation_vision_uncertainty,
                        ),
                    )

                if self.should_log:
                    # Multitag results don't have best and alternates
                    self.best_log.setPose(pose)
                    self.alt_log.setPose(pose)
            else:
                self.has_multitag = False
                for target in results.getTargets():
                    # filter out likely bad targets
                    if target.getPoseAmbiguity() > 0.25:
                        continue

                    poses = estimate_poses_from_apriltag(self.robot_to_camera, target)
                    if poses is None:
                        # tag doesn't exist
                        continue

                    best, alt, self.last_pose_z = poses
                    pose = choose_pose(
                        best,
                        alt,
                        self.chassis.get_pose(),
                    )

                    self.field_pos_obj.setPose(pose)
                    self.chassis.estimator.addVisionMeasurement(
                        pose,
                        timestamp,
                        (
                            self.linear_vision_uncertainty,
                            self.linear_vision_uncertainty,
                            self.rotation_vision_uncertainty,
                        ),
                    )

                    if self.should_log:
                        self.best_log.setPose(best)
                        self.alt_log.setPose(alt)

    @feedback
    def sees_target(self):
        return time.monotonic() - self.last_recieved_timestep < self.TIMEOUT


def estimate_poses_from_apriltag(
    robot_to_camera: Transform3d, target: PhotonTrackedTarget
) -> tuple[Pose2d, Pose2d, float] | None:
    tag_id = target.getFiducialId()
    tag_pose = apriltag_layout.getTagPose(tag_id)
    if tag_pose is None:
        return None

    best_pose = objectToRobotPose(
        tag_pose, target.getBestCameraToTarget(), robot_to_camera
    )
    alternate_pose = objectToRobotPose(
        tag_pose, target.getAlternateCameraToTarget(), robot_to_camera
    )
    return best_pose.toPose2d(), alternate_pose.toPose2d(), best_pose.z


def get_target_skew(target: PhotonTrackedTarget) -> float:
    tag_to_cam = target.getBestCameraToTarget().inverse()
    return math.atan2(tag_to_cam.y, tag_to_cam.x)


def choose_pose(best_pose: Pose2d, alternate_pose: Pose2d, cur_robot: Pose2d) -> Pose2d:
    """Picks either the best or alternate pose estimate"""
    best_dist = best_pose.translation().distance(cur_robot.translation())
    alternate_dist = (
        alternate_pose.translation().distance(cur_robot.translation())
        * VisualLocalizer.BEST_POSE_BIAS
    )

    if best_dist < alternate_dist:
        return best_pose
    else:
        return alternate_pose
