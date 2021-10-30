from setuptools import setup

# DO NOT EXECUTE THIS SCRIPT MANUALLY OR YOU MAY BREAK YOUR ROS INSTALLATION.
# CATKIN WILL EXECUTE THE SCRIPT FOR YOU

setup(
    version='0.0.0',        # Matches project version in package.xml
    packages=['controls'],  # Matches 'package()' directive in CMakeLists.txt
    package_dir={'': 'src'}
)