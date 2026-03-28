import os

from setuptools import find_packages, setup


package_name = "mushr_rhc"


def package_data_files(package_name, directory):
    data_files = []
    for root, _, files in os.walk(directory):
        if "__pycache__" in root.split(os.sep):
            continue
        if not files:
            continue
        install_dir = os.path.join("share", package_name, root)
        file_paths = [
            os.path.join(root, file_name)
            for file_name in files
            if not file_name.endswith((".pyc", ".pyo"))
        ]
        if file_paths:
            data_files.append((install_dir, file_paths))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src"),
    py_modules=["rhcbase", "rhcdebug", "rhcnode", "rhctensor", "utils"],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *package_data_files(package_name, "launch"),
        *package_data_files(package_name, "maps"),
        *package_data_files(package_name, "resource"),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "torch",
        "networkx",
        "scipy",
        "scikit-learn",
    ],
    zip_safe=True,
    maintainer="sg",
    maintainer_email="sg@todo.todo",
    description="Receding Horizon Controller base package",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rhcnode = rhcnode:main",
            "rhcdebug = rhcdebug:main",
        ],
    },
)
