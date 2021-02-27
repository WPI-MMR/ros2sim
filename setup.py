from setuptools import find_packages, setup


setup(name='ros2sim',
      version='0.0.1',
      packages=find_packages(),
      install_requires=['pyserial', 'numpy'],
      extras_require={
        'test': ['parameterized']
      }
)
