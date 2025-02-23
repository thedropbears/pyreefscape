import math
from dataclasses import dataclass
from typing import ClassVar

import wpilib
import wpiutil.log
import wpiutil.wpistruct
from magicbot import feedback, tunable
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath import objectToRobotPose
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation3d,
)
from wpimath.interpolation import TimeInterpolatableRotation2dBuffer

from components.chassis import ChassisComponent
from utilities.caching import HasPerLoopCache, cache_per_loop
from utilities.game import APRILTAGS, apriltag_layout
from utilities.rev import configure_through_bore_encoder
from utilities.scalers import scale_value


@wpiutil.wpistruct.make_wpistruct
@dataclass
class VisibleTag:
    WPIStruct: ClassVar

    tag_id: int
    relative_bearing: Rotation2d
    range: float


@dataclass
class ServoOffsets:
    neutral: Rotation2d
    full_range: Rotation2d


class VisualLocalizer(HasPerLoopCache):
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    # Give bias to the best pose by multiplying this const to the alt dist
    BEST_POSE_BIAS = 1.2

    # Time since the last target sighting we allow before informing drivers
    TIMEOUT = 1.0  # s

    CAMERA_FOV = math.radians(
        65
    )  # photon vision says 69.8, but we are being conservative
    CAMERA_MAX_RANGE = 4.0  # m

    add_to_estimator = tunable(True)
    should_log = tunable(True)

    last_pose_z = tunable(0.0, writeDefault=False)
    linear_vision_uncertainty = tunable(0.30)
    rotation_vision_uncertainty = tunable(0.6)

    linear_vision_uncertainty_multi_tag = tunable(0.05)
    rotation_vision_uncertainty_multi_tag = tunable(0.05)

    reproj_error_threshold = 0.1

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        # Position of the camera relative to the center of the robot
        turret_pos: Translation3d,
        # The turret rotation at its neutral position (ie centred).
        turret_rot: Rotation2d,
        # The camera relative to the turret (ie without servo rotation)
        camera_offset: Translation3d,
        # The camera pitch on the mount, relative to horizontal
        camera_pitch: float,
        servo_id: int,
        servo_offsets: ServoOffsets,
        encoder_id: int,
        encoder_offset: Rotation2d,
        # Encoder rotations at min and max of desired rotation range
        rotation_range: tuple[Rotation2d, Rotation2d],
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
        chassis: ChassisComponent,
    ) -> None:
        super().__init__()
        self.camera = PhotonCamera(name)
        self.encoder = wpilib.DutyCycleEncoder(encoder_id, math.tau, 0)
        configure_through_bore_encoder(self.encoder)
        # Offset of encoder in radians when facing forwards (the desired zero)
        # To find this value, manually point the camera forwards and record the encoder value
        # This has nothing to do with the servo - do it by hand!!
        self.encoder_offset = encoder_offset

        # To find the servo offsets, command the servo to neutral in test mode and record the encoder value
        # Repeat for full range
        self.servo_offsets = servo_offsets

        relative_rotations = [(r - encoder_offset).radians() for r in rotation_range]
        self.min_rotation = min(relative_rotations)
        self.max_rotation = max(relative_rotations)

        self.servo = wpilib.Servo(servo_id)
        self.pos = turret_pos
        self.robot_to_turret = Transform3d(turret_pos, Rotation3d(turret_rot))
        self.robot_to_turret_2d = Transform2d(turret_pos.toTranslation2d(), turret_rot)
        self.turret_to_camera = Transform3d(
            camera_offset, Rotation3d(roll=0.0, pitch=camera_pitch, yaw=0.0)
        )
        self.turret_rotation_buffer = TimeInterpolatableRotation2dBuffer(2.0)

        self.last_timestamp = -1.0
        self.last_recieved_timestep = -1.0
        self.best_log = field.getObject(name + "_best_log")
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

    @cache_per_loop
    def relative_bearing_to_best_cluster(self) -> Rotation2d:
        tags = self.visible_tags()
        if len(tags) == 0:
            return Rotation2d(0.0)
        relative_bearings = [tag.relative_bearing for tag in tags]
        relative_bearings.sort(key=Rotation2d.radians)
        for offset in range(len(relative_bearings) - 1, 0, -1):
            bearing_pairs = zip(relative_bearings, relative_bearings[offset:])
            for pair in bearing_pairs:
                if abs((pair[0] - pair[1]).radians()) < self.CAMERA_FOV:
                    return (pair[1] - pair[0]) * 0.5 + pair[0]
        # If we get here there are no pairs, so choose the closest
        tags.sort(key=lambda v: v.range)
        return tags[0].relative_bearing

    @feedback
    @cache_per_loop
    def visible_tags(self) -> list[VisibleTag]:
        tags_in_view = []

        robot_pose = self.chassis.get_pose()
        turret_pose = robot_pose.transformBy(self.robot_to_turret_2d)

        for tag in APRILTAGS:
            turret_to_tag = (
                tag.pose.toPose2d().translation() - turret_pose.translation()
            )
            relative_bearing = turret_to_tag.angle() - turret_pose.rotation()
            distance = turret_to_tag.norm()
            relative_facing = tag.pose.toPose2d().rotation() - turret_to_tag.angle()
            if (
                self.min_rotation <= relative_bearing.radians() <= self.max_rotation
                and abs(relative_facing.degrees()) > 100
                and distance < self.CAMERA_MAX_RANGE
            ):
                # Test for relative facing is more than 90 degrees because we don't want to be too
                # close to parallel to the tag
                tags_in_view.append(VisibleTag(tag.ID, relative_bearing, distance))

        return tags_in_view

    @feedback
    def desired_turret_rotation(self) -> Rotation2d:
        # Read encoder angle and account for offset
        return self.relative_bearing_to_best_cluster()

    def turret_to_servo(self, turret: Rotation2d) -> Rotation2d:
        return turret - (self.servo_offsets.neutral - self.encoder_offset)

    @property
    def turret_rotation(self) -> Rotation2d:
        return self.raw_encoder_rotation() - self.encoder_offset

    def robot_to_camera(self, timestamp: float) -> Transform3d:
        turret_rotation = self.turret_rotation_buffer.sample(timestamp)
        if turret_rotation is None:
            return self.robot_to_turret
        r2t = self.robot_to_turret.translation()
        t2c = self.turret_to_camera.translation().rotateBy(Rotation3d(turret_rotation))
        trans = r2t + t2c
        rot = (
            self.robot_to_turret.rotation()
            .rotateBy(Rotation3d(turret_rotation))
            .rotateBy(self.turret_to_camera.rotation())
        )
        return Transform3d(trans, rot)

    def zero_servo_(self) -> None:
        # ONLY CALL THIS IN TEST MODE!
        # This is used to put the servo in a neutral position to record the encoder value at that point
        self.servo.set(0.5)

    def full_range_servo_(self) -> None:
        # ONLY CALL THIS IN TEST MODE!
        # This is used to put the servo to the full range position to record the encoder value at that point
        self.servo.set(1.0)

    def execute(self) -> None:
        self.servo.set(
            scale_value(
                self.turret_to_servo(self.desired_turret_rotation()).radians(),
                (self.servo_offsets.neutral - self.encoder_offset).radians(),
                (self.servo_offsets.full_range - self.encoder_offset).radians(),
                0.5,
                1.0,
            )
        )

        self.turret_rotation_buffer.addSample(
            wpilib.Timer.getFPGATimestamp(), self.turret_rotation
        )

        all_results = self.camera.getAllUnreadResults()
        for results in all_results:
            # if results didn't see any targets
            if not results.getTargets():
                return

            # if we have already processed these results
            timestamp = results.getTimestampSeconds()

            if timestamp == self.last_timestamp:
                return
            self.last_recieved_timestep = wpilib.Timer.getFPGATimestamp()
            self.last_timestamp = timestamp

            camera_to_robot = self.robot_to_camera(timestamp).inverse()

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
            else:
                self.has_multitag = False
                for target in results.getTargets():
                    # filter out likely bad targets
                    if target.getPoseAmbiguity() > 0.25:
                        continue

                    poses = estimate_poses_from_apriltag(
                        self.robot_to_camera(results.getTimestampSeconds()), target
                    )
                    if poses is None:
                        # tag doesn't exist
                        continue

                    pose, _, __ = poses

                    self.field_pos_obj.setPose(pose)
                    self.chassis.estimator.addVisionMeasurement(
                        pose,
                        timestamp,
                        (
                            self.linear_vision_uncertainty_multi_tag,
                            self.linear_vision_uncertainty_multi_tag,
                            self.rotation_vision_uncertainty_multi_tag,
                        ),
                    )

                    if self.should_log:
                        self.best_log.setPose(pose)

    @feedback
    def sees_target(self):
        return (
            wpilib.Timer.getFPGATimestamp() - self.last_recieved_timestep < self.TIMEOUT
        )


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
