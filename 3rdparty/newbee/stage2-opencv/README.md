﻿0. 这是一个基于opencv的项目，你要做的首先是配置好opencv环境并编译（推荐使用release模式编译，否则会跑的很慢，另外此项目使用了多线程，如果你是在linux系统下编译此项目需要自己改下cmakelists）。若你成功编译运行后能看到一个窗口，这个窗口中有两个随机旋转的标靶。你只需要修改todo.cpp文件里的内容，width和height是窗口的大小，你可以改成别的大小；stage代表你完成的阶段；imgroot应根据你可执行文件和img文件夹的相对路径修改。
0. solve函数提供了一个Mat&对象，这个mat是最后窗口展示的内容，你可以直接在此对象上修改。在这个阶段，你需要通过opencv把那两个随机旋转的标靶持续框出来。solve函数会自动调用，每次传入的mat对象是无修改的。
0. 当你完成了上一小题后，你首先要把stage改为2再进行此题。这时编译运行代码你会看到除了旋转标靶外还有随机生成的高速小球，你所需要做的就是再把这些小球给框出来。