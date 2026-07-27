from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_camera_device_can_sleep_until_visual_handoff():
    source = (PACKAGE / 'src' / 'car_camera_node.cpp').read_text()

    assert 'declare_parameter<bool>("enabled", true)' in source
    assert '"~/set_enabled"' in source
    assert 'std_srvs::srv::SetBool' in source
    assert 'if (!enabled_ || !cap_.isOpened())' in source
    assert 'cap_.release()' in source


def test_board_launch_exposes_camera_gate():
    launch = (PACKAGE / 'launch' / 'board_parking.launch.py').read_text()

    assert "DeclareLaunchArgument(" in launch
    assert "'camera_enabled', default_value='true'" in launch
    assert "'enabled': ParameterValue(" in launch
    assert "LaunchConfiguration('camera_enabled')" in launch
