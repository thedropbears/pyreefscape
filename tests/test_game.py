import typing

from wpimath import geometry

from utilities import game


def test_get_fiducial_pose() -> None:
    # Check the types are correct
    for tag_id in typing.get_args(game.TagId):
        pose = game.get_fiducial_pose(tag_id)
        assert isinstance(pose, geometry.Pose3d)
