# 模糊控制基础练习

| 文件 | 内容 |
|---|---|
| [blur.m](blur.m) | 手写模糊化、规则推理和加权平均输出，演示一阶系统控制 |
| [blur_control.slx](blur_control.slx) | 使用 `myFis.fis` 的 Simulink 模型 |
| [myFis.fis](myFis.fis) | Mamdani 系统：两个输入、一个输出、9 条规则 |

## 使用

切换 MATLAB 当前文件夹到本目录：

```matlab
blur                            % 查看手写算法及响应图
open_system('blur_control.slx')  % 打开 Simulink 模型
```

`blur.m` 是独立算法练习，并非模型的专用初始化脚本。`blur_control.slx` 按文件名读取 `myFis.fis`，请将二者保持在同一目录并从本目录运行。FIS 模型需要 Simulink 和 Fuzzy Logic Toolbox。

编辑模糊规则：

```matlab
fuzzyLogicDesigner('myFis.fis')
```

本次仅归档，未重新验证原有模型仿真。三角隶属度模糊 PID 是另一个示例，见[使用说明](../../PID控制/模糊PID/README.md)。

[返回项目导航](../../README.md)
