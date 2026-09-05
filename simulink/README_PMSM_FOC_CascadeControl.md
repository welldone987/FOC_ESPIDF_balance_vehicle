# PMSM FOC 三级串级控制模型

`PMSM_FOC_CascadeControl.slx` 对应参考图中的位置环、速度环、dq 电流环、Clarke/Park、反 Park、平均值 SVPWM、PMSM 和速度/位置反馈。

运行前执行 `init_pmsm_foc_cascade.m`。当前电机与负载参数是 `UNVERIFIED` 仿真占位值；它们只用于检查拓扑、信号方向和闭环可运行性，不代表现有平衡车电机的实测结论。

模型边界：

- SVPWM 是平均值调制器，不包含功率器件开关沿、死区或母线纹波；
- PMSM 是 dq 连续平均模型，abc 端口用于连接控制结构；
- 控制器是离散实现，默认电流、速度和位置环使用同一基础采样时间，便于首版结构验证；
- `Position_Ref` 使用弧度，`Speed` 使用 rad/s，电流使用 A，电压使用 V。

