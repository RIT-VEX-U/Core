# Robot Template

A template for starting a new project using RIT-VEX-U's core library.

Includes:
- Core library (as a subtree)
- Eigen linear algebra library (as a submodule)
- A github action to test compilation

## Setup
1. Fork the repository

![image](https://github.com/user-attachments/assets/bbfe6035-c14f-4e0a-9378-e42846c40522)

2. Clone the repository - `git clone --recurse-submodules git@github.com:RIT-VEX-U/ForkTemplate.git` 
    - Alternatively, one can `git clone git@github.com:RIT-VEX-U/ForkTemplate.git` then execute `git submodule init` followed by `git submodule update`
3. Open the repository in VSCode (or any other editor if you don't need the VSCode extension)

If you do not wish to contribute to core, you can also just download the zip. However, if you intend to commit upstream, inititializing with the above steps make it much easier.

### Troubleshooting
If simply opening the folder in VSCode does not work, try navigating to the VEX extension panel and importing this folder
