from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- BURASI KRİTİK: Launch ve Parametre dosyalarını sisteme tanıtıyoruz ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        # ------------------------------------------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hasan',
    maintainer_email='hasan@todo.todo',
    description='Mecanum Kinematics Assignment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- BURASI KRİTİK: Node ismini ve çalışacak fonksiyonu belirtiyoruz ---
            'mecanum_kinematics_node = mecanum_kinematics.mecanum_kinematics_node:main',
        ],
    },
)