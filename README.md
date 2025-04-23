# GKD 电视一体电控部分代码
- 本项目是为哨兵开发的电视一体代码的衍生泛用版本。

  ![logo](img/logo.png)

## Documentation
[doc](doc/doc.md "doc")

[hardware_manager](doc/hardware_manager.md "doc")

## Building
Using bash like command line to build
- GNU/Make
```
$ make $(robot_type) -j8
$ make run
```

- CMake
```
$ mkdir build
$ cd build && make -j 8
$ make run
```

## 使用 XMake 编译
XMake 是一个基于 Lua 的轻量级跨平台构建工具，使用 xmake.lua 维护项目构建，相比 makefile/CMakeLists.txt，配置语法更加简洁直观，对新手非常友好，短时间内就能快速入门，能够让用户把更多的精力集中在实际的项目开发上。

> 关于如何安装 XMake, 请参考 [XMake 安装教程](https://xmake.io/#/zh-cn/guide/installation)

### 1.设置编译选项

```bash
xmake -f --type=<type> --mode=<debug/release>
```

其中 `<type>` 为 `sentry`、`infantry`、`hero` 之一，`mode` 表示当前以 `debug` 模式编译还是 `release` 模式。

在使用过程中，你可以随时运行上面的目录来修改编译选项。

### 2.编译

```bash
xmake build
```

在构建过程中会要求安装第三方库，输入 `y` 即可。

在编译结束后，你可以在输出文件夹中找到编译结果，生成的二进制文件名与第一步中设置的 `type` 一致。输出文件夹位于 `build` 目录下，并按照以下规则命名：[平台]\\[架构]\\[编译模式]。例如，假设在x86的linux上以 release模式编译，那么输出的文件位于 build\linux\x86_64\release下。