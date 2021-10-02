import sys

import yaml


def main():
    workflow_path = sys.argv[1]
    with open(workflow_path) as f:
        workflow = yaml.load(f, Loader=yaml.CLoader)
    steps = workflow["jobs"]["build"]["steps"]
    depsteps = [s for s in steps if s["name"] == "Install Dependencies"]
    assert len(depsteps) == 1
    depstep = depsteps[0]
    code = depstep["run"]
    code = code.replace(
        "${{ matrix.pyVer }}", "${CSW_PYTHON}"
    ).replace(
        "${{ matrix.ros }}", "[ROS version]"
    )
    print(code.strip())


if __name__ == "__main__":
    main()
