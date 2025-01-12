import math
import time

import wpilib
import wpiutil.log
from magicbot import feedback, tunable
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from wpimath import objectToRobotPose
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Transform3d, Translation3d

from components.chassis import ChassisComponent
from utilities.functions import clamp
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

    SERVO_MAX_ANGLE = math.radians(135)

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
        # The camera rotation.
        rot: Rotation3d,
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
        chassis: ChassisComponent,
    ) -> None:
        self.camera = PhotonCamera(name)
        # Assuming channel is 0 for encoder
        self.encoder = wpilib.DutyCycleEncoder(1)
        # Offset of encoder in radians when facing forwards (the desired zero)
        self.encoder_offset = 3.33
        self.servo = wpilib.Servo(0)
        self.pos = pos
        self.robot_to_servo = Transform3d(pos, rot)
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
    def read_encoder(self) -> float:
        # reads value from encoder (presumed to be value from 0 - 1) and converts to radians by multiplying by 2pi
        encoder_result_radians = self.encoder.get() * math.tau
        return encoder_result_radians

    @feedback
    def bearing_to_closest_tag(self) -> float:
        # initialise default variables
        # closest tag, distance, robot position
        distance = 1.8e1038

        for tag in APRILTAGS:
            robot_to_tag = tag.pose.toPose2d() - self.chassis.get_pose()
            if robot_to_tag.translation().norm() < distance:
                closest_tag = tag

        diff = (closest_tag.pose.toPose2d() - self.chassis.get_pose()).translation()

        yaw = math.atan2(diff.Y(), diff.X())
        return yaw

    @property
    def turret_rotation(self) -> float:
        return self.read_encoder() - self.encoder_offset

    def execute(self) -> None:
        # Read encoder angle
        # account for offset
        servo_to_camera = Transform3d(
            Translation3d(), Rotation3d(0, 0.0, self.turret_rotation)
        )

        desired_servo_angle = (
            self.bearing_to_closest_tag()
            - self.chassis.get_rotation().radians()
            - self.robot_to_servo.rotation().Z()
        )
        clamped_angle = clamp(
            desired_servo_angle, -self.SERVO_MAX_ANGLE, self.SERVO_MAX_ANGLE
        )
        self.servo.set(
            scale_value(
                clamped_angle, -self.SERVO_MAX_ANGLE, self.SERVO_MAX_ANGLE, 0.0, 1.0
            )
        )

        # reset self.camera_to_robot
        robot_to_camera = self.robot_to_servo + servo_to_camera
        camera_to_robot = robot_to_camera.inverse()

        results = self.camera.getLatestResult()
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
                self.best_log.setPose(
                    Pose2d(p.best.x, p.best.y, p.best.rotation().toRotation2d())
                )
                self.alt_log.setPose(
                    Pose2d(p.alt.x, p.alt.y, p.alt.rotation().toRotation2d())
                )
        else:
            self.has_multitag = False
            for target in results.getTargets():
                # filter out likely bad targets
                if target.getPoseAmbiguity() > 0.25:
                    continue

                poses = estimate_poses_from_apriltag(robot_to_camera, target)
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
