from setuptools import find_packages, setup

package_name = 'goal_time_field'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', ['config/goal_time_field.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'goal_time_field_train = goal_time_field.training.train:main',
          'goal_time_field_evaluate = goal_time_field.training.evaluate:main',
          'goal_time_field_plot_slice = goal_time_field.training.plot_slice:main',
          'goal_time_field_node = goal_time_field.ros.inference_node:main']})
