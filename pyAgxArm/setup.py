from setuptools import setup, find_packages
from pathlib import Path

about = {}
exec(Path("pyAgxArm/version.py").read_text(encoding="utf-8"), about)

setup(
    name='pyAgxArm',
    version=about["__version__"],
    license='LGPL-3.0-only',
    packages=find_packages(
        include=['pyAgxArm', 'pyAgxArm.*'],
    ),
    author='Agilex Robotics Co., Ltd.',
    author_email='',
    description='Pure teleop runtime subset for Nero arm and gripper CAN control',
    platforms=['Linux'],
    python_requires='>=3.6',
)
