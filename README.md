# AVAI Lab

This is the repository for the course "Autonomous Vehicles And Artificial Intelligence" (AVAI).

Take a look at the [Documentation](https://rub-avai.github.io/race-car-buggy-busters/)

## Structure

The respository contains:
- folders for ROS2 packages (`_ws` - suffix)
- the `avai_lab` python package
- [mkdocs](https://squidfunk.github.io/mkdocs-material/getting-started/) documentation

All important documentation and information will be compiled in the mkdocs pages.
You can find them in the `docs` folder.

## Developer Installation

In order to use this repository you will need [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (ships with python3.10).

First make sure you have the latest and greatest installation tools:

```sh
python3 -m pip install --upgrade pip
pip install --upgrade packaging setuptools
```

Install the `avai_lab` package as editable. The `[docs]` installs the optional dependencies for `mkdocs`

```sh
pip install -e avai_lab[docs]
```

Now go and build the ROS2 `test_package`.

```sh
cd race_car_ws
colcon build
source install/setup.bash
```

Now you should be able to view `mkdocs`
(In the repositories base directory)
```sh
mkdocs serve
```
