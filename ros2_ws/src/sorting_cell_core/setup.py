from setuptools import find_packages, setup


setup(
    name="sorting_cell_core",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sorting_cell_core"]),
        ("share/sorting_cell_core", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML==6.0.3"],
    zip_safe=True,
    maintainer="sorting-cell maintainer",
    maintainer_email="maintainer@example.invalid",
    description="Hardware-independent core for the UR10e sorting cell",
    license="Proprietary",
)
