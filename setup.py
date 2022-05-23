from distutils.core import setup

setup (
    version='0.0.0',
    scripts=['scripts/aruco.py', 'scripts/arm_grab.py', 'scripts/client_test.py'],
    packages=['dmarm_grab'],
    package_dir={'':'scripts'}
)