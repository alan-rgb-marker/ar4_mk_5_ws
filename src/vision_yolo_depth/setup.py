from setuptools import find_packages, setup

package_name = 'vision_yolo_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alan',
    maintainer_email='danrotech99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_camera = vision_yolo_depth.vision:main',
            'shelf_pose_detect = vision_yolo_depth.shelf_pose_detect_service:main',
            'demo_shelf_pose_detect = vision_yolo_depth.demo_shelf_pose_detect:main',
            'demo_real_depth_camera = vision_yolo_depth.demo_real_depth_camera:main',
            'demo_yolo = vision_yolo_depth.demo_yolo:main',
            'demo_shelf_pose_detect_with_yolo = vision_yolo_depth.demo_shelf_pose_detect_with_yolo:main',
            'main_service = vision_yolo_depth.main_service:main',
        ],
    },
)
