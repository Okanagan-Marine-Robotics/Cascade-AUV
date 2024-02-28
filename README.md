# Marine Design Software

## Overview

This repository contains the software for the marine design team. The software is divided into two main parts: the Jetson Orin Nano and the ESP32. The Jetson Orin Nano is the main computer on the AUV and is responsible for high-level decision making. The ESP32 is responsible for low-level control and sensor interfacing.

## Documentation

Documentation is very important to us. We have created an in-house documentation system that allows us to easily create and maintain documentation. The documentation is written in markdown and is hosted on GitHub Pages. To add documentation, simply create a new markdown file in the location you are working on. Give it a title and a description, and it will automatically be added to the docs.

The styling of the documentation is done using the [Docsify](https://docsify.js.org/) framework. This allows us to easily create a professional-looking documentation website with minimal effort.

> **Important**: The docs folder is just used for generation of the documentation. You should NEVER write any documentation in the docs folder! All documentation should be written in the folder that it is relevant to.

The script that generates the documentation is located in the root of the repository and is called `generatedocs.py`. This script finds all the markdown files in the repository copy's them to the docs folder and then generates the sidebar. The sidebar is generated by reading the markdown files and extracting the title. The script then writes the sidebar to the `_sidebar.md` file in the docs folder.

If you don't want a file to be included in the documentation, simply add a `.docsignore` file to the folder that the file is in. The `.docsignore` file will stop and files in that folder or subfolders from being included in the documentation.

> This may change in the future as we are always looking for ways to improve our documentation system.
