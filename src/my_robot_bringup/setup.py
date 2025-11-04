from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_bringup'
package_dir = os.path.join('src', package_name)  # ← パス修正用

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 絶対パスで登録（←ここが重要）
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(os.path.dirname(__file__), 'launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(os.path.dirname(__file__), 'config', '*'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join(os.path.dirname(__file__), 'urdf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tatsu',
    maintainer_email='tatsu@todo.todo',
    description='Bringup package for biped robot simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

