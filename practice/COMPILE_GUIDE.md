# 🛠️ 编译和测试指南

## 📋 快速开始

### 第一步：测试IO管理器（Step 1）
```bash
# 编译
g++ -std=c++17 test_step1.cpp -o test_step1

# 运行测试
./test_step1
```

### 第二步：测试回调系统（Step 2）
```bash
# 编译
g++ -std=c++17 test_step2.cpp -o test_step2

# 运行测试
./test_step2
```

### 第三步：测试电机驱动系统（Step 3）
```bash
# 编译（注意需要pthread库）
g++ -std=c++17 -pthread test_step3.cpp -o test_step3

# 运行测试
./test_step3
```

## 🔧 编译要求

- **C++17标准**：所有练习都使用C++17特性
- **pthread库**：Step 3需要多线程支持
- **GCC/Clang**：推荐使用现代编译器

## 📝 常见编译错误及解决方案

### 错误1：未定义的函数
```
undefined reference to 'register_callback'
```
**解决方案**：您需要在step2_callback_system.hpp中实现所有TODO标记的函数。

### 错误2：模板错误
```
error: no matching function for call to 'forward'
```
**解决方案**：检查step1中的完美转发实现，确保包含了`<utility>`头文件。

### 错误3：多线程错误
```
undefined reference to 'pthread_create'
```
**解决方案**：使用`-pthread`编译选项。

## 🎯 测试说明

### 测试通过标志
- ✅ 绿色勾号：测试通过
- 🎉 庆祝emoji：完成所有测试
- 💡 技能列表：显示您掌握的技能

### 测试失败标志
- ❌ 红色X：测试失败
- 💡 常见问题检查：调试建议

## 🚀 学习顺序

1. **先实现再测试**：完成一个step的所有TODO后再运行测试
2. **逐步调试**：如果测试失败，逐个检查每个函数的实现
3. **理解错误信息**：仔细阅读编译错误和运行时错误
4. **不要急于看答案**：尝试自己解决问题，这是学习的关键

## 💡 调试技巧

### 1. 使用详细的编译选项
```bash
g++ -std=c++17 -Wall -Wextra -g -O0 test_step1.cpp -o test_step1
```

### 2. 使用调试器
```bash
gdb ./test_step1
(gdb) run
(gdb) bt  # 查看调用堆栈
```

### 3. 添加调试输出
```cpp
std::cout << "调试：函数被调用" << std::endl;
```

## 🏆 完成标志

当您看到这些消息时，说明您已经掌握了相应的技能：

- **Step 1完成**：🎉 IO管理器测试通过
- **Step 2完成**：🎉 回调系统测试通过  
- **Step 3完成**：🎉🎉🎉 电机驱动系统测试通过

## 📞 遇到问题？

如果遇到无法解决的问题：
1. 检查是否实现了所有TODO项
2. 确认头文件包含正确
3. 验证函数签名与声明一致
4. 检查是否有语法错误

**记住：实践是最好的老师！不要害怕犯错，每个错误都是学习的机会。** 💪 