from setuptools import Extension, setup

kinematics_library_module = Extension("kinematics_library", sources=[], language="c")

setup(
    name="kinematics_library",
    version="1.1.4",
    description="Python bindings for the C kinematics library.",
    include_package_data=True,
    ext_modules=[kinematics_library_module],
)
