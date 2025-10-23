## 变更

*date: 2025/10/23*

*Committer: Xiayh*



## 总览

本次更改添加了部分目录，并重新规划了整个项目的目录结构（3rdparty除外）



目前 build 是可以通过的，中间有部分 warning 不知道以后会不会修。

由于改动较大，可能会导致奇怪的错误？仍然需要<u>实际运行验证</u>。

## 

## 添加

- 添加了 `log` 目录

- 添加了 `script` 目录

- 添加了 `include/control` 目录

- 添加了`include/robot_controller`目录

- 添加了 `include/io` 目录

- 添加了 `include/shoot`目录

## 移动

- 将`draw.py`移动到`scripts`目录下

- 将`fric_log.txt`、`log.txt` 移动到`log`目录下

- 将 `src/hardware/dji_motor.cc` 移动到 `src/device`目录下

- 将 `src/hardware/can.cc` 移动到 `src/io`目录下

- 将 `include/pid_controller.hpp`、`include/ramp.hpp`、`include/controller.hpp`移动到 `include/control`目录下

- 将 `include/robot_controller.hpp`移动到`include/robot_controller/robot_controller.hpp`目录下

- 将 `include/shoot_config.hpp` 移动到 `include/configs`目录下

- 将 `include/dji_motor.hpp` 移动到 `include/device/dji_motor.hpp`目录下

- 将 `include/can.hpp`、`include/io.hpp`、 `include/socket_interface.hpp`、 `include/serial_interface.hpp` 移动到 `include/io`目录下

- 将 `include/user_lib.hpp`、`include/utils.hpp`、`include/types.hpp`移动到`include/utils`目录下

- 将 `include/macro_helpers.hpp` 移动到 `include/utils`目录下

- 将 `include/shoot.hpp`、`include/bullet_solver.hpp`移动到 `include/shoot`目录下

- 将 `include/hardware_callback.hpp` 移动到 `include/io/io_callback.hpp`(见下文修改 - 修改了部分命名)
  
  

## 修改

- 修改了部分路径配置（不一定修改到所有路径）
  
  - **Ln 7 in draw.py**
  
  - **Ln 10 in CMakeLists.txt**
  
  - **Ln 67 in src/shoot/shoot.cc**
  
  - **Ln 233 in src/chassis/power_controller.cc**

- 修改了部分命名
  
  - 将 `include/hardware_callback.hpp` 更名为 `io_callback.hpp`(参考`doc/hardware_manager.md`) (为什么我找不到`hardware_manager.hpp`?)并移动`include/io`目录下
    
    - 同时修改了对这个头文件的 include,包括以下文件:
      
      - `include/io/can.hpp`
      
      - `include/io/serial_interface.hpp`
      
      - `include/io/socket_interface.hpp`
      
      - `doc/hardware_manager.md`

- 修改了`CMakeLists.txt`
  
  - 将新目录添加至 `include_directories`中

- 修改了 `xmake.lua`
  
  - 将新目录添加至 `add_includedirs`中
