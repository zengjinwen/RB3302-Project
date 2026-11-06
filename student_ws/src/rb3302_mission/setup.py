from glob import glob
from setuptools import find_packages, setup
name = 'rb3302_mission'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'README.md']),
                  ('share/'+name+'/launch', glob('launch/*.py')),
                  ('share/'+name+'/config', glob('config/*.yaml')),
                  ('share/'+name+'/rviz', glob('rviz/*.rviz')),
                  ],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='RB3302 project team', maintainer_email='rb3302@example.com',
      description='Four-direction grid A-star with straight driving and in-place turns.',
      license='Apache-2.0', entry_points={'console_scripts': ['mission = rb3302_mission.node:main',
          'mission_arc = rb3302_mission.arc_node:main',
          'mission_obstacle = rb3302_mission.obstacle_node:main',
          'legacy_base = rb3302_mission.base_driver:main']})
