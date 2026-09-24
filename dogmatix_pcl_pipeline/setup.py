from setuptools import setup, find_packages

setup(
    name='dogmatix_pcl_pipeline',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/dogmatix_pcl_pipeline']),
        ('share/dogmatix_pcl_pipeline', ['package.xml']),
        ('share/dogmatix_pcl_pipeline/launch', ['launch/pipeline.launch.py']),
        ('share/dogmatix_pcl_pipeline/config', ['config/pipeline_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dogmatix',
    maintainer_email='dev@dogmatix.local',
    description='Point cloud processing for Dogmatix 3D Model Generator',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'pointcloud_processor = dogmatix_pcl_pipeline.pointcloud_processor:main',
            'floorplan_generator = dogmatix_pcl_pipeline.floorplan_generator:main',
            'pcl_pipeline_node = dogmatix_pcl_pipeline.pcl_pipeline_node:main',
        ],
    },
)
