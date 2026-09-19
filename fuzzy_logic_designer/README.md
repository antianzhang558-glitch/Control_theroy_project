# 模糊逻辑设计器练习

[mamdanitype1.fis](mamdanitype1.fis) 是已有 Mamdani 模糊推理系统，包含两个输入、一个输出和 9 条规则，使用重心法解模糊。

安装 Fuzzy Logic Toolbox 后，将 MATLAB 当前文件夹切换到本目录，执行：

```matlab
fuzzyLogicDesigner('mamdanitype1.fis')
```

可在设计器中查看隶属度函数、规则和推理曲面。它与 `fuzzy_control/myFis.fis` 是分别保存的文件，修改一份不会自动同步另一份。

[返回项目导航](../README.md)
