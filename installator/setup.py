from setuptools import setup

setup(name='hrim',
      version='0.3',
      license='Apache-2.0',
      packages=['hrim','hrim.scripts'],
      description="HRIM command line utility",
      long_description='''
      The Hardware Robot Information Model or HRIM for short, is a common interface that facilitates interoperability among different vendors of robot hardware components with the purpose of building modular robots.
      HRIM focuses on the standardization of the logical interfaces between robot modules, designing a set of rules that each device has to meet in order to achieve interoperability.
      ''',
      url="https://github.com/erlerobot/HRIM",
      entry_points={
          'console_scripts': [
              'hrim = hrim.__main__:main'
          ]
      },
    install_requires=[
        'lxml',
      ],
      keywords=['HRIM','H-ROS', 'modular', 'ros2', 'robot operating system'],
      )
