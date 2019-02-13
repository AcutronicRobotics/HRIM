from setuptools import setup

setup(name='hrim',
      version='0.0.1',
      packages=['hrim','hrim.scripts'],
      entry_points={
          'console_scripts': [
              'hrim = hrim.__main__:main'
          ]
      },
      )
