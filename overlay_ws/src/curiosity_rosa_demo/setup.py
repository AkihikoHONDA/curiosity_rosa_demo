from glob import glob
from setuptools import setup

package_name = "curiosity_rosa_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        f"{package_name}.agent",
        f"{package_name}.sim",
        f"{package_name}.adapter",
        f"{package_name}.tools",
        f"{package_name}.trace",
        f"{package_name}.viz",
        f"{package_name}.infra",
        f"{package_name}.domain",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/srv", glob("srv/*.srv")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy", "opencv-python", "rich"],
    zip_safe=True,
    maintainer="Curiosity ROSA Demo",
    maintainer_email="dev@example.com",
    description="Curiosity ROSA demo package.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulator_node = curiosity_rosa_demo.sim.simulator_node:main",
            "adapter_node = curiosity_rosa_demo.adapter.adapter_node:main",
            "agent_node = curiosity_rosa_demo.agent.agent_node:main",
        ]
    },
)
