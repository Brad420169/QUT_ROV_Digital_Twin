"""Scenario selection reaches Stonefish without modifying environment files."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import SetParameter

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "ros_nodes/modular_architecture"))
import rov_launcher_gui as gui


@pytest.mark.parametrize("scenario", ["main_rov.scn", "main_rov_tri_bouyancy.scn", "main_rov_lil_tri_block.scn", "main_rov_square_block.scn"])
def test_gui_passes_selected_scenario_only_to_sim(monkeypatch, scenario):
    monkeypatch.setattr(gui, "find_optional_venv", lambda: None)
    assert f"--mode sim --rov-scenario {scenario}" in gui.build_shell_command("sim", scenario)
    assert "--rov-scenario" not in gui.build_shell_command("real", scenario)


@pytest.mark.parametrize("scenario", ["main_rov.scn", "main_rov_tri_bouyancy.scn", "main_rov_lil_tri_block.scn", "main_rov_square_block.scn"])
@pytest.mark.parametrize("environment", ["ocean_environment", "pool_environment"])
def test_launch_parameter_resolves_rov_include(scenario, environment):
    spec = importlib.util.spec_from_file_location("rov_launch", PACKAGE / "launch/launch_rov.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    group = next(action for action in description.entities if isinstance(action, GroupAction))
    parameter = next(action for action in group.get_sub_entities() if isinstance(action, SetParameter))
    context = LaunchContext()
    context.launch_configurations.update(rov_scenario=scenario, environment=environment)
    parameter.execute(context)
    selected = Path(dict(context.launch_configurations["global_params"])["rov_scenario"])
    assert selected.name == scenario
    assert selected.is_file()
    assert ET.parse(selected).find("robot") is not None
    includes = ET.parse(PACKAGE / "scenarios" / f"{environment}.scn").findall("include")
    rov = next(item for item in includes if item.get("file") == "$(param rov_scenario)")
    assert next(arg for arg in rov.findall("arg") if arg.get("name") == "vehicle_name").get("value") == "qut_rov"


@pytest.mark.parametrize("exit_code", [0, 7])
def test_terminal_waits_after_success_or_failure(monkeypatch, tmp_path, exit_code):
    import subprocess

    setup = tmp_path / "setup.bash"
    setup.write_text(f"ros2() {{ return {exit_code}; }}\n")
    monkeypatch.setattr(gui, "ROS_SETUP", str(setup))
    monkeypatch.setattr(gui, "WORKSPACE_SETUP", str(setup))
    monkeypatch.setattr(gui, "find_optional_venv", lambda: None)
    process = subprocess.Popen(
        ["bash", "-c", gui.build_shell_command("sim")],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert process.stdout.readline() == "\n"
        message = process.stdout.readline()
        assert f"exit code {exit_code}" in message
        assert "Press Enter" in message
        assert process.poll() is None
        process.communicate(input="\n", timeout=3)
        assert process.returncode == exit_code
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
