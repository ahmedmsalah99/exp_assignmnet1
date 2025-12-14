# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
sys.path.insert(0, os.path.abspath('../../src/aruco_scanner/aruco_scanner'))


project = 'Aruco_Scanner_Node'
copyright = '2025, Ahmed_Salah-Mazen_Atta-Mohamed_Ismail'
author = 'Ahmed_Salah-Mazen_Atta-Mohamed_Ismail'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
]
autodoc_mock_imports = [
    'rclpy',
    'geometry_msgs',
    'nav_msgs',
    'sensor_msgs',
    'aruco_opencv_msgs',
    'cv_bridge',
    'cv2',
    'numpy',
]


templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
