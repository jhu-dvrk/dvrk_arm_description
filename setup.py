from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_arm_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/arms", [
            str(path) for path in sorted(Path("arms").glob("*.yaml"))
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Anton Deguet",
    maintainer_email="anton.deguet@jhu.edu",
    description="Canonical dVRK arm descriptions and their Python configuration API.",
    license="Apache-2.0",
)
