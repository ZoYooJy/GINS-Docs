# Note

坐标系：

- 导航系：`n`
- 惯性系：`i`
- 地心地固系：`e`
- 车载系：`v`
- IMU系：`b`

IMU安装规则：

- x -- forward -- roll
- y -- right -- pitch
- z -- down -- yaw

欧拉角旋转顺序：

- Z-Y-X

- $C = C_z \cdot C_y \cdot C_x$

$$
\begin{align}
\mathbf{C}_x(\phi) &= \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix} \\

\mathbf{C}_y(\theta) &= \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{bmatrix} \\

\mathbf{C}_z(\psi) &= \begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}

\end{align} \tag{1.1}
$$




# `Coarse estimation`



## `roll/pitch`

假设：

- 对于车载组合导航，IMU与车载捷联，且假设车载与导航坐标系处于同一水平面，因此计算得到的水平姿态角即为IMU相对于车载的水平安装角；



惯导比力方程：
$$
\dot{v}_{en}^{n} = C_{b}^{n} f_{sf}^{b} - (2 \omega_{ie}^{n} + \omega_{en}^{n}) \times v_{en}^{n} + g^{n} \tag{2.1.1}
$$


静态状态下，载体的线运动及其导数为0，比力方程简化为：
$$
0 = C_{b}^{n} f_{sf}^{b} + g^{n} => f_{sf}^{b} = - (C_{b}^{n})^{T} \cdot g^{n} \tag{2.1.2}
$$


矩阵形式（y：yaw，p：pitch，r：roll）：
$$
\begin{bmatrix}
f_{sfx}^{b} \\ f_{sfy}^{b} \\ f_{sfz}^{b}
\end{bmatrix}

= -

\begin{bmatrix}
cy \cdot cp & cy \cdot sp \cdot sr - sy \cdot cr & cy \cdot sp \cdot cr + sy \cdot sr \\
sy \cdot cp & sy \cdot sp \cdot sr + cy \cdot cr & sy \cdot sp \cdot cr - cy \cdot sr \\
-sp			& cp \cdot sr						 & cp \cdot cr
\end{bmatrix}
^{T}

\begin{bmatrix}
0 \\ 0 \\ G_{0}
\end{bmatrix} \tag{2.1.3}
$$
展开得：
$$
\begin{align}
sp 			&= \frac{f_{sfx}^{b}}{G_{0}} \\
cp \cdot sr &= \frac{-f_{sfy}^{b}}{G_{0}} \\
cp \cdot cr &= \frac{-f_{sfz}^{b}}{G_{0}} \\
\end{align} \tag{2.1.4}
$$
则水平姿态角为：
$$
\begin{align}
roll &= atan2(-f_{sfy}^{b}, -f_{sfz}^{b}) \\
pitch &= asin(\frac{f_{sfx}^{b}}{G_{0}}) \\
\end{align} \tag{2.1.5}
$$




## `yaw`

**note：**

- 对于载车，其行驶速度方向一般沿载体纵轴方向（正前方），根据卫星测量速度矢量，可以估计载体纵轴相对于地理北向的方位角；
- 通过在水平方向上做短时间**直线加速**机动实现方位角估计（本质：在水平方向上提供一个用于确定方位的观测量）；



粗略安装时，为了简化计算，忽略哥氏加速度和对地向心加速度，比力方程近似为：
$$
\begin{align}
\dot{v}^{n} &\approx C_{b}^{n} f_{sf}^{b} + g^{n} \\
&= C_{v}^{n} C_{h}^{v} C_{b}^{h} f_{sf}^{b} + g^{n} \\
&= C_{v}^{n} C_{h}^{v} f_{sf}^{h} + g^{n} \\
\end{align} \tag{2.2.1}
$$
两边同时乘以$C_{n}^{v}$：
$$
\begin{align}
C_{n}^{v} \cdot \dot{v}^{n} &= C_{h}^{v} f_{sf}^{h} + C_{n}^{v} g^{n} \\
\dot{v}^{v} &= C_{h}^{v} f_{sf}^{h} + g^{v} \\
\end{align} \tag{2.2.2}
$$
其中，

- h：当地水平坐标系；
- $C_{n}^{v}$：在**前进、直线、无侧滑、速度足够**时，通过gnss NED速度计算得到，$A_n^v \approx \begin{bmatrix}0 & 0 & atan2(v_E, v_N)\end{bmatrix}$
- $C_{h}^{v} = \begin{bmatrix}cy & -sy & 0 \\ sy & cy & 0 \\ 0 & 0 & 1\\\end{bmatrix}$，对应航向安装角
- $\dot{v}^{n} = \frac{ v_{GNSS}^{n}(t_{k}) - v_{GNSS}^{n}(t_{k-1}) }{ t_{k} - t_{k-1} }$，通过相邻2个时刻卫星导航速度的平均变化量近似估计；
- v系与n系处于同一水平面，因此$g^v = \begin{bmatrix} 0 \\ 0 \\ G_0 \end{bmatrix}$



转换为矩阵形式：
$$
\begin{bmatrix}
\dot{{v}}_{F}^{v} \\ \dot{{v}}_{R}^{v} \\ \dot{{v}}_{D}^{v}
\end{bmatrix}

=

\begin{bmatrix}
cy & -sy & 0 \\ sy & cy & 0 \\ 0 & 0 & 1\\
\end{bmatrix}

\begin{bmatrix}
{ {f}}_{sfx}^{h} \\  { {f}}_{sfy}^{h} \\  { {f}}_{sfz}^{h}
\end{bmatrix}

+

\begin{bmatrix}
0 \\ 0 \\ G_{0}
\end{bmatrix} \tag{2.2.3}
$$
对于x、y轴水平分量：
$$
\begin{align}
\dot{{v}}_{F}^{v} &= cy \cdot {{f}}_{sfx}^{h} - sy \cdot {{f}}_{sfy}^{h}  \\

\dot{{v}}_{R}^{v} &= sy \cdot {{f}}_{sfx}^{h} + cy \cdot {{f}}_{sfy}^{h}  \\
\end{align} \tag{2.2.4}
$$
解得：
$$
\begin{align}
sy &= \frac{\dot{{v}}_{R}^{v} \cdot {{f}}_{sfx}^{h} - \dot{{v}}_{F}^{v} \cdot {{f}}_{sfy}^{h}}{ {{{f}}_{sfx}^{h}}^2 +  {{{f}}_{sfy}^{h}}^2} \\

cy &= \frac{\dot{{v}}_{F}^{v} \cdot {{f}}_{sfx}^{h} + \dot{{v}}_{R}^{v} \cdot {{f}}_{sfy}^{h}}{ {{{f}}_{sfx}^{h}}^2 +  {{{f}}_{sfy}^{h}}^2} \\
\end{align} \tag{2.2.5}
$$
因此：
$$
\begin{align}
tan(y) &= \frac{\dot{{v}}_{R}^{v} \cdot {{f}}_{sfx}^{h} - \dot{{v}}_{F}^{v} \cdot {{f}}_{sfy}^{h}}{\dot{{v}}_{F}^{v} \cdot {{f}}_{sfx}^{h} + \dot{{v}}_{R}^{v} \cdot {{f}}_{sfy}^{h}} \\

yaw &= atan(\frac{\dot{{v}}_{R}^{v} \cdot {{f}}_{sfx}^{h} - \dot{{v}}_{F}^{v} \cdot {{f}}_{sfy}^{h}}{\dot{{v}}_{F}^{v} \cdot {{f}}_{sfx}^{h} + \dot{{v}}_{R}^{v} \cdot {{f}}_{sfy}^{h}}) \\
\end{align} \tag{2.2.6}
$$


工程上，取多组数据构造线性方程组进行求解。

基于x、y轴水平分量的矩阵形式，建立观测方程：
$$
\begin{align}
\begin{bmatrix} {f}_{sfx}^{h} & -{f}_{sfy}^{h} \\ {f}_{sfy}^{h} & {f}_{sfx}^{h} \end{bmatrix}
\cdot
\begin{bmatrix}
cy \\ sy
\end{bmatrix}
=
\begin{bmatrix} \dot{{v}}_{F}^{v} \\ \dot{{v}}_{R}^{v} \end{bmatrix}
\end{align} \tag{2.2.7}
$$


令：
$$
\begin{align}
x &= \begin{bmatrix}
cy \\ sy
\end{bmatrix} \\
y &= \begin{bmatrix} \dot{{v}}_{F}^{v} \\ \dot{{v}}_{R}^{v} \end{bmatrix} \\
H &= \begin{bmatrix} {f}_{sfx}^{h} & -{f}_{sfy}^{h} \\ {f}_{sfy}^{h} & {f}_{sfx}^{h} \end{bmatrix} \\
\end{align} \tag{2.2.8}
$$


假设有k组数据，则矩阵y为`2k*1`，矩阵H为`2k*2`，则有：
$$
\begin{align}
H^T &= \begin{bmatrix} {f}_{sfx}^{h} & {f}_{sfy}^{h} \\ -{f}_{sfy}^{h} & {f}_{sfx}^{h} \end{bmatrix} \\

H^T \cdot H &= \begin{bmatrix} {{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2 & 0 \\ 0 & {{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2 \end{bmatrix} \\

(H^T \cdot H)^{-1} &= \begin{bmatrix} \frac{1}{ {{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2 } & 0 \\ 0 & \frac{1}{ {{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2 } \end{bmatrix} \\

H^T \cdot y &= \begin{bmatrix} \dot{{v}}_{F}^{v} \cdot {{f}}_{sfx}^{h} + \dot{{v}}_{R}^{v} \cdot {{f}}_{sfy}^{h} \\ \dot{{v}}_{R}^{v} \cdot {{f}}_{sfx}^{h} - \dot{{v}}_{F}^{v} \cdot {{f}}_{sfy}^{h} \end{bmatrix} \\
\end{align} \tag{2.2.9}
$$
因此，对线性方程组求解得到：
$$
\begin{align}
x &= (H^T \cdot H)^{-1} \cdot (H^T \cdot y) \\
&= \begin{bmatrix} \frac{\dot{{v}}_{F}^{v} \cdot {{f}}_{sfx}^{h} + \dot{{v}}_{R}^{v} \cdot {{f}}_{sfy}^{h}}{{{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2} \\ \frac{\dot{{v}}_{R}^{v} \cdot {{f}}_{sfx}^{h} - \dot{{v}}_{F}^{v} \cdot {{f}}_{sfy}^{h}}{{{f}_{sfx}^{h}}^2 +  {{f}_{sfy}^{h}}^2} \end{bmatrix} \\
\end{align} \tag{2.2.10}
$$
最终有：
$$
yaw = atan2(x[1], x[0]) \tag{2.2.11}
$$





# `Fine estimation`

核心思想：**通过寻找一组安装角，使得IMU观测到的速度矢量在经过坐标变换后，在车体坐标系中主要沿前进方向**。



水平变换：
$$
\mathbf{C}_{yx} = \mathbf{C}_y(\theta)\mathbf{C}_x(\phi) =
\begin{bmatrix}
\cos\theta & \sin\theta\sin\phi & \sin\theta\cos\phi \\
0 & \cos\phi & -\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix} \tag{3.1}
$$

$$
\mathbf{C}_b^v = \mathbf{C}_z(\psi)\mathbf{C}_{yx} =
\begin{bmatrix}
c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\
s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\
-s\theta & c\theta s\phi & c\theta c\phi
\end{bmatrix} \tag{3.2}
$$


假设：

- 速度矢量约束：车载坐标系下，速度矢量主要沿前进方向，即$\mathbf{v}^v \approx \begin{bmatrix} v & 0 & 0 \end{bmatrix}^{T}$

- 对于IMU观测速度，经过安装角变换后，车载坐标系中观测的速度为：$\mathbf{v}^v = \mathbf{C}_b^v \mathbf{v}^b \approx \begin{bmatrix} v & 0 & 0 \end{bmatrix}^{T}$



## 航向角闭合求解

对于中间坐标系h，有：
$$
\mathbf{v}^h = \mathbf{C}_y(\theta)\mathbf{C}_x(\phi) v^b = \mathbf{C}_{yx}\mathbf{v}^b = \begin{bmatrix}
v_x^h \\
v_y^h \\
v_z^h
\end{bmatrix} \tag{3.1.1}
$$
将IMU的观测速度转换到车载系：
$$
\begin{align}
v^v &= \mathbf{C}_z(\psi)\mathbf{C}_y(\theta)\mathbf{C}_x(\phi) v^b \\
&= \mathbf{C}_z(\psi) v^h \\
\end{align} \tag{3.1.2}
$$
横向速度分量为：
$$
v_y^v = [\mathbf{C}_z(\psi) v^h]_y = \sin\psi \cdot v_x^h + \cos\psi \cdot v_y^h \tag{3.1.3}
$$
令$v_y^v = 0$：
$$
\tan\psi = -\frac{v_y^h}{v_x^h} \tag{3.1.4}
$$
 即：
$$
\psi = \tan^{-1}\left(-\frac{v_y^h}{v_x^h}\right) \tag{3.1.5}
$$




## 横滚和俯仰角修正量求解

目标：最小化横向和垂向速度（$min|v_y^v|$，$min|v_z^v|$），使得：$v_y^v = 0, v_z^v = 0$

残差：

$$
\begin{align}
\mathbf{\epsilon} &= C_{bv} \cdot \mathbf{v}^b - [v_f, 0, 0]^T \\
&= \begin{bmatrix}
c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\
s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\
-s\theta & c\theta s\phi & c\theta c\phi
\end{bmatrix}

\cdot

\begin{bmatrix}
v_x^b \\ v_y^b \\ v_z^b
\end{bmatrix}

-

\begin{bmatrix}
v^f \\ 0 \\ 0
\end{bmatrix}

\end{align} \tag{3.2.1}
$$
横向和垂向残差：
$$
\begin{align}
\mathbf{\epsilon}_y &= s\psi c\theta \cdot v_x^b + (s\psi s\theta s\phi + c\psi c\phi) \cdot v_y^b + (s\psi s\theta c\phi - c\psi s\phi) \cdot v_z^b \\

\mathbf{\epsilon}_z &= -s\theta \cdot v_x^b + c\theta s\phi \cdot v_y^b + c\theta c\phi \cdot v_z^b \\

\end{align} \tag{3.2.2}
$$
对于残差方程，在当前估计值附近进行一阶泰勒展开：

$$
\boldsymbol{\epsilon}(\phi + \Delta\phi, \theta + \Delta\theta) \approx \boldsymbol{\epsilon}(\phi, \theta) + \frac{\partial \boldsymbol{\epsilon}}{\partial \phi} \Delta\phi + \frac{\partial \boldsymbol{\epsilon}}{\partial \theta} \Delta\theta \tag{3.2.3}
$$
期望残差为0：

$$
\frac{\partial \mathbf{\epsilon}}{\partial \phi} \Delta\phi + \frac{\partial \mathbf{\epsilon}}{\partial \theta} \Delta\theta = -\mathbf{\epsilon} \tag{3.2.4}
$$
对于横向和垂向残差，有：
$$
\frac{\partial \mathbf{\epsilon}_y}{\partial \phi} \Delta\phi + \frac{\partial \mathbf{\epsilon}_y}{\partial \theta} \Delta\theta = -\mathbf{\epsilon}_y \\

\frac{\partial \mathbf{\epsilon}_z}{\partial \phi} \Delta\phi + \frac{\partial \mathbf{\epsilon}_z}{\partial \theta} \Delta\theta = -\mathbf{\epsilon}_z \tag{3.2.5}
$$
残差分量分别对$\phi$和$\theta$求偏导：
$$
\begin{align}

\frac{\partial\epsilon_y}{\partial_\phi} &= (s\psi s\theta c\phi - c\psi s\phi) \cdot v_y^b + (-s\psi s\theta s\phi - c\psi c\phi) \cdot v_z^b \\

\frac{\partial\epsilon_y}{\partial_\theta} &= -s\psi s\theta \cdot v_x^b + s\psi c\theta s\phi \cdot v_y^b + s\psi c\theta c\phi \cdot v_z^b \\

\frac{\partial\epsilon_z}{\partial_\phi} &= c\theta c\phi \cdot v_y^b - c\theta s\phi \cdot v_z^b \\

\frac{\partial\epsilon_z}{\partial_\theta} &= -c\theta \cdot v_x^b - s\theta s\phi \cdot v_y^b - s\theta c\phi \cdot v_z^b \\

\end{align} \tag{3.2.6}
$$
对横向和垂向残差建立线性方程组，克莱姆法则求解校正量：
$$
\begin{align}
\begin{cases}
A_1\Delta\phi + B_1\Delta\theta = C_1 \\
A_2\Delta\phi + B_2\Delta\theta = C_2
\end{cases}

=>

\begin{bmatrix} A_1 & B_1 \\ A_2 & B_2 \end{bmatrix} \begin{bmatrix} \Delta\phi \\ \Delta\theta \end{bmatrix} = \begin{bmatrix} C_1 \\ C_2 \end{bmatrix} \\
\\
=>

\Delta\phi = \frac{C_1B_2 - C_2B_1}{A_1B_2 - A_2B_1}, \quad \Delta\theta = \frac{C_1A_2 - C_2A_1}{B_1A_2 - B_2A_1} \\
\end{align} \tag{3.2.7}
$$
其中，
$$
\begin{align}

A_1 &= \frac{\partial\epsilon_y}{\partial_\phi} = (s\psi s\theta c\phi - c\psi s\phi) \cdot v_y^b + (-s\psi s\theta s\phi - c\psi c\phi) \cdot v_z^b \\
B_1 &= \frac{\partial\epsilon_y}{\partial_\theta} = -s\psi s\theta \cdot v_x^b + s\psi c\theta s\phi \cdot v_y^b + s\psi c\theta c\phi \cdot v_z^b \\
C_1 &= -s\psi c\theta \cdot v_x^b - (s\psi s\theta s\phi + c\psi c\phi) \cdot v_y^b - (s\psi s\theta c\phi - c\psi s\phi) \cdot v_z^b \\
\\
A_2 &= \frac{\partial\epsilon_z}{\partial_\phi} = c\theta c\phi \cdot v_y^b - c\theta s\phi \cdot v_z^b \\
B_2 &= \frac{\partial\epsilon_z}{\partial_\theta} = -c\theta \cdot v_x^b - s\theta s\phi \cdot v_y^b - s\theta c\phi \cdot v_z^b \\
C_2 &= s\theta \cdot v_x^b - c\theta s\phi \cdot v_y^b - c\theta c\phi \cdot v_z^b \\

\end{align} \tag{3.2.8}
$$





## 最优解选择

定义误差函数为：
$$
E(C_b^v) = \sum_{k=1}^{N} \frac{\| \mathbf{v}_{k, y/z} \|}{\| \mathbf{v}_k \|} \tag{3.3.1}
$$


几何意义：衡量速度向量和前向轴之间的夹角，
$$
\frac{\sqrt{v_y^2 + v_z^2}}{\sqrt{v_x^2 + v_y^2 + v_z^2}} = \sqrt{1 - \left(\frac{v_x}{\|v\|}\right)^2} = \sin(\alpha) \tag{3.3.2}
$$
原则：

- 尺度不变性：对速度大小不敏感
- 旋转敏感性：对安装角误差敏感
- 统计稳定性：多组数据平均提高鲁棒性



选择条件：

$$
nk_{\text{min}} = \underset{nk}{\text{argmin}} \left( \sum_{k=0}^{N-1} \frac{\sqrt{(v_{y,k}^v)^2 + (v_{z,k}^v)^2}}{\sqrt{(v_{x,k}^v)^2 + (v_{y,k}^v)^2 + (v_{z,k}^v)^2}} \right) \tag{3.3.3}
$$








## 安装精度

### 评估

- DOP[0]：总体精度（侧向+垂向）
- DOP[1]：侧向安装角精度（主要反映偏航角误差）
- DOP[2]：垂向安装角精度（主要反映俯仰角误差）



### DOP与安装角误差的关系

$\hat{C}_b^v$：估计安装角，$C_b^v$：真实安装角。



误差旋转矩阵为：
$$
\delta C = \hat{C}_b^v \cdot (C_b^v)^T ≈ I + \begin{bmatrix}
0 & -\delta\psi & \delta\theta \\
\delta\psi & 0 & -\delta\phi \\
-\delta\theta & \delta\phi & 0
\end{bmatrix} \tag{3.4.1}
$$


速度误差为：
$$
\delta \mathbf{v}^v ≈ \begin{bmatrix}
0 & -\delta\psi & \delta\theta \\
\delta\psi & 0 & -\delta\phi \\
-\delta\theta & \delta\phi & 0
\end{bmatrix} \cdot \mathbf{v}^v \tag{3.4.2}
$$


对于车载运动，$\mathbf{v}^v \approx \begin{bmatrix} v^f & 0 & 0 \end{bmatrix}^{T}$，则：
$$
\begin{align}
\delta v_y &≈ \delta\psi \cdot v_f \\
\delta v_z &≈ -\delta\theta \cdot v_f \\
\end{align} \tag{3.4.3}
$$
即，
$$
\begin{align}
\text{DOP}_{\text{lateral}} &≈ |\delta\psi| \\
\text{DOP}_{\text{vertical}} &≈ |\delta\theta| \\
\end{align} \tag{3.4.4}
$$



