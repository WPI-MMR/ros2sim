from setuptools import setup


setup(name='ros2sim',
      version='0.0.1',
      install_requires=['pyserial'],
      extras_require={
        'test': ['parameterized']
      }
)
