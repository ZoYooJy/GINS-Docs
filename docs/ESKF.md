**统一约定：**

**$X = \hat{X} + {\delta}X$**
- $X$：真实值，物理世界中系统实际所处的状态，永远未知、无法直接获取；
- $\hat{X}$：名义值，算法内部维护的对真实状态的当前最优估计，由预测（IMU积分）和更新（观测修正）得到；
	- 对于IMU，名义值由“原始输出 - 名义零偏”、再积分得到；
- $\delta X$：误差状态，待估计的小量；

$$
名义值 = 上一时刻反馈校正后的名义状态X_{k-1}^{+} \xrightarrow{INS机械编排（时间更新）} 预测名义状态X_{k}^{-} \xrightarrow{量测更新、反馈校正} 校正后的名义状态X_{k}^{+} \tag{P1}
$$


**误差状态向量：**
$$
\delta{X} =
\begin{bmatrix}
\delta{r^n} & \delta{v^n} & \phi & \delta{b_g^b} & \delta{b_a^b}
\end{bmatrix}^T \tag{P2}
$$
- $\delta r^n$：ECEF位置误差投影到NED导航系下，$m$
- $\delta v^n$：NED导航系下，$m/s$
- $\phi$：真实导航系$n$==主动旋转==到计算导航系$n'$的等效旋转矢量${\phi}_{n}^{n'}$
	- 姿态误差的定义（n系/b系、旋转方向）影响后续建模时的`+ / -`


**过程噪声向量：**
$$
\omega =
\begin{bmatrix}
n_g^{b} & n_a^{b} & n_{b_g}^{b} & n_{b_a}^{b} \\
\end{bmatrix}^T \tag{P3}
$$

**$\psi$角误差模型 => 连续的系统矩阵F => 近似的离散状态转移矩阵$\Phi$**
$$
\Phi _{k} = e^{F(t_k) \cdot \Delta t} \approx I + F(t_k) \cdot \Delta t \tag{P4}
$$

|         |                     连续时间模型（物理世界）                     |                       离散时间模型（计算机世界）                       |
| :-----: | :--------------------------------------------------: | :-------------------------------------------------------: |
|   公式    | $\delta \dot{X} = F \cdot \delta X + G \cdot \omega$ | $\delta X_{k+1} = \Phi \cdot \delta X_{k} + \omega{_{k}}$ |
|  物理含义   |                        瞬时变化率                         |                          一段时间的累积                          |
|  状态矩阵   |                        系统矩阵F                         |            状态转移矩阵$\Phi = e^{F \cdot \Delta t}$            |
|  噪声矩阵   |                   $Q_c$ （功率谱密度PSD）                   |                           $Q_d$                           |
|  零偏：RW  |                       $F = 0$                        |                        $\Phi = I$                         |
| 零偏：FOGM |              $F = - \frac{1}{\tau{_c}}$              |          $\Phi = e^{-\frac{\Delta t}{\tau _c}}$           |
- $\Phi$和$Q_d$是$F$和$Q_c$在一个采样周期内==积分（累积）==的结果
- $Q_c$：“噪声的功率”；    $Q_d$：“采样周期内累积的噪声总能量”


**注：**
- 传统捷联惯导误差方程采用“估计值 = 真实值 + 估计误差”的形式进行建模；
- 误差状态 = - 估计误差


**坐标系：**
- 导航系：`n`
- 惯性系：`i`
- 地心地固系：`e`
- 车载系：`v`
- IMU系：`b`


**IMU安装规则：**
- x -- forward -- roll
- y -- right -- pitch
- z -- down -- yaw

**欧拉角旋转顺序：**
- Z-Y-X
- $C = C_z \cdot C_y \cdot C_x$

**四元数：**
- Hamilton format
- $q = \begin{bmatrix} q_w & q_x & q_y & q_z \end{bmatrix}^{T}$


**坐标系旋转与坐标变换**

定义：
- ${\phi} _{A}^{B}$ 定义为A系==主动旋转==到B系的等效旋转矢量，对应的坐标系旋转矩阵（主动旋转矩阵）为$R_{A}^{B}$
- $C_A^B / q_A^B$ 定义为将同一个矢量从A系变换到B系的坐标变换矩阵
- ${\phi} / R$ 负责“主动旋转”语义，$C / q$ 负责“坐标变换”语义
$$
\begin{align}
{\phi} &= {\phi}_{n}^{n'} \\
R_{n}^{n'} &= (C_{n}^{n'})^{-1} = \mathbf{Exp}({\phi})  = \exp({\phi}^{\wedge})  \\
C_{n}^{n'} &= \mathbf{Exp}(-{\phi}) = \exp(-{\phi}^{\wedge}) \\
C_{n'}^{n} &= \mathbf{Exp}(+{\phi}) = \exp(+{\phi}^{\wedge}) \\
q_{n}^{n'} &= \mathbf{Exp}(-{\phi}) = \exp(-{\phi}^{\wedge}) \\
q_{n'}^{n} &= \mathbf{Exp}(+{\phi}) = \exp(+{\phi}^{\wedge}) \\

\end{align} \tag{P5}
$$


# 1. 非线性模型


## 1.1 运动模型

真实状态运动模型：
$$
\begin{align}
\dot{X} = \mathbf{f}({X}, \mu, \omega) \\
\end{align} \tag{1.1.1}
$$
- ${X}$：真实状态
- $\mu$：控制输入，组合导航中表征为**角速度测量和比力测量**
- $\omega$：过程噪声，假设为高斯白噪声
	- $\omega \sim \mathbf{N}(0, Q_c)$
	- $Q_c$：连续时间过程噪声的功率谱密度矩阵
	- $\mathbf{E}[\omega _c (t)] = 0,\quad \mathbf{E}[\omega _c (t) \cdot \omega _c^T (\tau)] = Q_c(t) \cdot \delta(t - \tau)$


## 1.2 观测模型

$$
\begin{align}
z = \mathbf{h}(X) + \nu \\
\end{align} \tag{1.2.1}
$$
- $z$：观测值
- $\nu$：观测噪声，假设为高斯白噪声，$\nu \sim \mathbf{N}(0, R)$

# 2. 线性模型

## 2.1 连续时间误差状态运动模型

名义状态不含过程噪声，由非线性模型通过控制输入进行传播：
$$
\dot{\hat{X}} = \mathbf{f}(\hat{X}, \mu, 0) \\ \tag{2.1.1}
$$

对真实状态运动模型在名义状态处一阶泰勒展开，得到误差状态的连续时间线性微分方程：

$$
\begin{align}

\mathbf{f}({X}, \mu, \omega)
&\approx
\mathbf{f}(\hat{X}, \mu, 0) + F(t) \cdot {\delta}X(t) + G(t) \cdot \omega(t) \\
F(t) &= \frac{\partial \mathbf{f}}{\partial X}|_{\hat{X}, \mu, 0} \\
G(t) &= \frac{\partial \mathbf{f}}{\partial \omega}|_{\hat{X}, \mu, 0} \\

\\

\mathbf{f}({X}, \mu, \omega) - \mathbf{f}(\hat{X}, \mu, 0)
&\approx
F(t) \cdot {\delta}X(t) + G(t) \cdot \omega(t) \\

\dot{X} - \dot{\hat{X}}
&\approx
F(t) \cdot {\delta}X(t) + G(t) \cdot \omega(t) \\

{\delta}\dot{X}(t)
&\approx
F(t) \cdot {\delta}X(t) + G(t) \cdot \omega(t) \\

\end{align} \tag{2.1.2}
$$

### 2.1.1 连续系统矩阵: F



- **零偏建模为随机游走**

完整的==工程==连续系统矩阵为：
$$
F
=
\begin{bmatrix}

-(\hat{\omega}_{en}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

(\hat{v}_{en}^{n})^{\wedge} \cdot (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r} &
(\hat{v}_{en}^{n})^{\wedge} \cdot \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n} \\

-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) &
-\hat{M}_{\omega{_{en}}/v} &
-(\hat{\omega}_{in}^{n})^{\wedge} &
-\hat{C}_{b}^{n} &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

\end{bmatrix} \tag{2.1.3}
$$

近似的==工程==连续系统矩阵为：
$$
\begin{align}
F
&=
\begin{bmatrix}

-(\hat{\omega}_{en}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

0_{3\times3} & - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega} _{en}^n)^{\wedge} & - (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n} \\

0_{3\times3} & 0_{3\times3} & -(\hat{\omega}_{in}^{n})^{\wedge} & -\hat{C}_{b}^{n} & 0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

\end{bmatrix}

\end{align} \tag{2.1.4}
$$



- **零偏建模为FOGM**

完整的==工程==连续系统矩阵为：
$$
F
=
\begin{bmatrix}

-(\hat{\omega}_{en}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

(\hat{v}_{en}^{n})^{\wedge} \cdot (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r} &
(\hat{v}_{en}^{n})^{\wedge} \cdot \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n} \\

-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) &
-\hat{M}_{\omega{_{en}}/v} &
-(\hat{\omega}_{in}^{n})^{\wedge} &
-\hat{C}_{b}^{n} &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & - \frac{1}{\tau{_{g,c}}} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & - \frac{1}{\tau{_{a,c}}} \cdot I_{3\times3} \\

\end{bmatrix} \tag{2.1.5}
$$

近似的==工程==连续系统矩阵为：
$$
\begin{align}
F
&=
\begin{bmatrix}

-(\hat{\omega}_{en}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

0_{3\times3} & - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega} _{en}^n)^{\wedge} & - (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n} \\

0_{3\times3} & 0_{3\times3} & -(\hat{\omega}_{in}^{n})^{\wedge} & -\hat{C}_{b}^{n} & 0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & - \frac{1}{\tau{_{g,c}}} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & - \frac{1}{\tau{_{a,c}}} \cdot I_{3\times3} \\

\end{bmatrix}

\end{align} \tag{2.1.6}
$$


#### 捷联惯导误差方程

> [!测量误差与零偏误差的关系]
> IMU原始输出：
> $$
> \begin{align}
> \widetilde{\omega}_{ib}^{b} &= {\omega}_{ib}^{b} + b_g^{b} + n_g^{b} \\
>  \widetilde{f}_{sf}^{b} &= {f}_{sf}^{b} + b_a^{b} + n_a^{b} \\
> \end{align}
> $$
> 
> 名义值 = 原始输出 - 名义零偏：
> $$
> \begin{align}
> \hat{\omega}_{ib}^{b} &= \widetilde{\omega}_{ib}^{b} - \hat{b}_g^{b} \\
> \hat{f}_{sf}^{b} &= \widetilde{f}_{sf}^{b} - \hat{b}_a^{b} \\
> \end{align}
> $$
> 
> 真实值 = 名义值 + 误差状态：
> $$
> \begin{align}
> {f}_{sf}^{b} &= \hat{f}_{sf}^{b} + \delta {f}_{sf}^{b} \\
> {\omega}_{ib}^{b} &= \hat{\omega}_{ib}^{b} + \delta {\omega}_{ib}^{b} \\
> {b}_g^{b} &= \hat{b}_g^{b} + \delta {b}_g^{b} \\
> {b}_a^{b} &= \hat{b}_a^{b} + \delta {b}_a^{b} \\
> \end{align}
> $$
> 
> 结合以上公式，得到：
> $$
> \begin{align}
> 
> \hat{\omega}_{ib}^{b} 
> &= [{\omega}_{ib}^{b} + b_g^{b} + n_g^{b}] - [{b}_g^{b} - \delta {b}_g^{b}] \\
> &= {\omega}_{ib}^{b} + \delta {b}_g^{b} + n_g^{b} \\
> {\omega}_{ib}^{b} - \hat{\omega}_{ib}^{b}
> &= -\delta {b}_g^{b} - n_g^{b} \\
> {\delta}{\omega}_{ib}^{b} &= -\delta {b}_g^{b} - n_g^{b} \\
> 
> \\
> 
> \hat{f}_{sf}^{b} &= [{f}_{sf}^{b} + b_a^{b} + n_a^{b}] - [{b}_a^{b} - \delta {b}_a^{b}] \\
> &= {f}_{sf}^{b} + \delta {b}_a^{b} + n_a^{b} \\
> {f}_{sf}^{b} - \hat{f}_{sf}^{b}
> &= -\delta {b}_a^{b} - n_a^{b} \\
> {\delta}{f}_{sf}^{b} &= -\delta {b}_a^{b} - n_a^{b} \\
> 
> \end{align}
> $$

##### 完整的误差传播相关推导


- 与位置微分方程类似，使用一阶泰勒展开更加自然的得到以**名义值**计算的误差表达式
- 直接求偏导得到的误差表达式，在实际应用中，使用**名义值**计算

###### 角速度

对于地球自转角速度$\omega _{ie}^{n}$和导航系转动角速度$\omega _{en}^{n}$，展开为矩阵形式：
$$
\begin{align}
\omega _{ie}^{n} &=
\begin{bmatrix}
0 \\ \omega _{ie} \cdot \cos{L} \\ \omega _{ie} \cdot \sin{L}
\end{bmatrix} _{ENU}
=
\begin{bmatrix}
\omega _{ie} \cdot \cos{L} \\ 0 \\ -\omega _{ie} \cdot \sin{L}
\end{bmatrix} _{NED} \\

\end{align} \tag{2.1.7}
$$
$$
\begin{align}
\omega _{en}^{n} &=
\begin{bmatrix}
- \frac{v_N}{R_M + h} \\ \frac{v_E}{R_N + h} \\ \frac{v_E \cdot \tan{L}}{R_N + h}
\end{bmatrix} _{ENU}
=
\begin{bmatrix}
\frac{v_E}{R_N + h} \\ - \frac{v_N}{R_M + h} \\ -\frac{v_E \cdot \tan{L}}{R_N + h}
\end{bmatrix} _{NED}
\end{align} \tag{2.1.8}
$$


分别直接求偏导：
$$
\begin{align}

\delta \omega _{ie}^{n} &=
\frac{\partial \omega _{ie}^{n}}{\partial {L}} \cdot \delta{L}
+ \frac{\partial \omega _{ie}^{n}}{\partial {\lambda}} \cdot \delta{\lambda}
+ \frac{\partial \omega _{ie}^{n}}{\partial {h}} \cdot \delta{h} \\

&=
\begin{bmatrix}
\omega _{ie} \cdot (-\sin{L}) \\ 0 \\ -\omega _{ie} \cdot \cos{L}
\end{bmatrix} _{NED}
\cdot \delta{L} \\

&=
\begin{bmatrix}
-\omega _{ie} \cdot \sin{L} & 0 & 0 \\
0 & 0 & 0 \\
-\omega _{ie} \cdot \cos{L} & 0 & 0 \\
\end{bmatrix} _{NED}

\cdot

\begin{bmatrix}
\delta{L} \\ \delta{\lambda} \\ \delta{h}
\end{bmatrix}
\\
&=
M_{\omega{_{ie}}/r} \cdot \delta{r} \\

\end{align} \tag{2.1.9}
$$

$$
\begin{align}
\delta \omega _{en}^{n}
&= \frac{\partial \omega _{en}^{n}}{\partial {L}} \cdot \delta{L}
+ \frac{\partial \omega _{en}^{n}}{\partial {\lambda}} \cdot \delta{\lambda}
+ \frac{\partial \omega _{en}^{n}}{\partial {h}} \cdot \delta{h}
+ \frac{\partial \omega _{en}^{n}}{\partial {v_N}} \cdot \delta{v_N}
+ \frac{\partial \omega _{en}^{n}}{\partial {v_E}} \cdot \delta{v_E}
+ \frac{\partial \omega _{en}^{n}}{\partial {v_D}} \cdot \delta{v_D} \\

&=

\begin{bmatrix}
0 \\ 0 \\ -\frac{v_E \cdot \sec^{2}{L}}{R_N + h} \\
\end{bmatrix}_{NED} \cdot \delta{L}

+ \begin{bmatrix}
-\frac{v_E}{(R_N + h)^2} \\ \frac{v_N}{(R_M + h)^2} \\ \frac{v_E \cdot \tan{L}}{(R_N + h)^2} \\
\end{bmatrix}_{NED} \cdot \delta{h}

+ \begin{bmatrix}
0 \\ -\frac{1}{R_M + h} \\ 0 \\
\end{bmatrix}_{NED} \cdot \delta{v_N}

+ \begin{bmatrix}
\frac{1}{R_N + h} \\ 0 \\ -\frac{\tan{L}}{R_N + h} \\
\end{bmatrix}_{NED} \cdot \delta{v_E} \\

&=

\begin{bmatrix}
0 & 0 & -\frac{v_E}{(R_N + h)^2} \\
0 & 0 & \frac{v_N}{(R_M + h)^2} \\
-\frac{v_E \cdot \sec^{2}{L}}{R_N + h} & 0 & \frac{v_E \cdot \tan{L}}{(R_N + h)^2} \\
\end{bmatrix}_{NED}

\cdot

\begin{bmatrix}
\delta{L} \\ \delta{\lambda} \\ \delta{h} \\
\end{bmatrix}

+

\begin{bmatrix}
0 & \frac{1}{R_N + h} & 0 \\
-\frac{1}{R_M + h} & 0 & 0 \\
0 & -\frac{\tan{L}}{R_N + h} & 0 \\
\end{bmatrix}_{NED}

\cdot

\begin{bmatrix}
\delta{v_N} \\ \delta{v_E} \\ \delta{v_D} \\
\end{bmatrix}

\\
&=
M_{\omega{_{en}}/r} \cdot \delta{r} + M_{\omega{_{en}}/v} \cdot \delta{v^n}


\end{align} \tag{2.1.10}
$$

###### 重力矢量

考虑Heiskanen垂线偏差的重力矢量方程为：
$$
\begin{align}

g^n
&=
\begin{bmatrix}
0 \\ -\beta{_3} \cdot h \cdot \sin{(2 \cdot L)} \\ -g_0 \cdot [1 + \beta \cdot \sin{^2}(L) - \beta{_1} \cdot \sin{^2}(2 \cdot L)] + \beta{_2} \cdot h \\
\end{bmatrix}_{ENU}
\\

&=
\begin{bmatrix}
-\beta{_3} \cdot h \cdot \sin{(2 \cdot L)} \\
0 \\
g_0 \cdot [1 + \beta \cdot \sin{^2}(L) - \beta{_1} \cdot \sin{^2}(2 \cdot L)] - \beta{_2} \cdot h \\
\end{bmatrix}_{NED}

\end{align} \tag{2.1.11}
$$

求偏导得：
$$
\begin{align}
\delta g^{n} &=
\frac{\partial g^{n}}{\partial {L}} \cdot \delta{L}
+ \frac{\partial g^{n}}{\partial {\lambda}} \cdot \delta{\lambda}
+ \frac{\partial g^{n}}{\partial {h}} \cdot \delta{h} \\

&=

\begin{bmatrix}
-2 \cdot \beta{_3} \cdot h \cdot \cos{2 \cdot L} \\
0 \\
g_0 \cdot \beta \cdot \sin{(2 \cdot L)} - 4 \cdot g_0 \cdot \beta{_1} \cdot \sin{(2 \cdot L)} \cdot \cos{(2 \cdot L)} \\
\end{bmatrix}_{NED} \cdot \delta{L}

+

\begin{bmatrix}
-\beta{_3} \cdot \sin{(2 \cdot L)} \\
0 \\
-\beta{_2} \\
\end{bmatrix}_{NED} \cdot \delta{h} \\

&=

\begin{bmatrix}
-2 \cdot \beta{_3} \cdot h \cdot \cos{2 \cdot L} & 0 & -\beta{_3} \cdot \sin{(2 \cdot L)} \\
0 & 0 & 0 \\
g_0 \cdot \beta \cdot \sin{(2 \cdot L)} - 4 \cdot g_0 \cdot \beta{_1} \cdot \sin{(2 \cdot L)} \cdot \cos{(2 \cdot L)} & 0 & -\beta{_2} \\
\end{bmatrix}_{NED}

\cdot

\begin{bmatrix}
\delta{L} \\ \delta{\lambda} \\ \delta{h} \\
\end{bmatrix}
\\
&= M_{g/r} \cdot \delta{r} \\

\end{align} \tag{2.1.12}
$$


##### 姿态


理想姿态矩阵为$C_b^n$，计算姿态矩阵（名义值）为$C_b^{n'}(\hat{C}_{b}^n)$，$n$作为参考系时，**$n$系到$n'$系**的等效旋转矢量为$\phi _{n}^{n'}$，即为==失准角误差==$\phi = \phi _{n}^{n'}$。
若$\phi$为小量，则有：
$$
\begin{align}
C_{n}^{n'} &= \mathbf{Exp}(-{\phi}) = \exp{(-{\phi}^{\wedge})} \approx I - \phi ^{\wedge} \\
C_{n'}^{n} &= \mathbf{Exp}(+{\phi}) = \exp{(+{\phi}^{\wedge})} \approx I + \phi ^{\wedge} \\
\end{align} \tag{2.1.13}
$$

$$
\begin{align}
C_{b}^{n} &= C_{n'}^{n} \cdot C_{b}^{n'} \\
C_{b}^{n'} &= C_{n}^{n'} \cdot C_{b}^{n} \\
\end{align} \tag{2.1.14}
$$

考虑==真实值 = 名义值 + 误差状态量==，有：
$$
\begin{align}
C_{b}^{n} &= (I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n} \\
C_{n}^{b} &= \hat{C}_{n}^{b} \cdot (I - \phi ^{\wedge}) \\
\end{align} \tag{2.1.15}
$$


真实姿态微分方程使用**名义值 + 误差状态量**进行表示：
$$
\begin{align}
\dot{{C}_{b}^n}
&= {C}_b^n \cdot ({\omega} _{ib}^b)^{\wedge} - ({\omega} _{in}^n)^{\wedge} \cdot {C}_b^n \\
&= (I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot (\hat{\omega} _{ib}^b + {\delta}{\omega} _{ib}^b)^{\wedge}
- (\hat{\omega} _{in}^n + {\delta}{\omega} _{in}^n)^{\wedge} \cdot (I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n} \\
\end{align} \tag{2.1.16}
$$

名义姿态微分方程：
$$
\begin{align}
\dot{\hat {C}_{b}^n}
&= \hat{C}_b^n \cdot (\hat{\omega} _{ib}^b)^{\wedge}
- (\hat{\omega} _{in}^n)^{\wedge} \cdot \hat{C}_b^n \\
\end{align} \tag{2.1.17}
$$


对$\ref{eq:2.1.15}$进行微分，并将$\ref{eq:2.1.17}$带入：
$$
\begin{align}

\dot{C}_{b}^{n}
&= (\dot{\phi})^{\wedge} \cdot \hat{C}_{b}^{n} + (I + \phi ^{\wedge}) \cdot \dot{\hat{C}}_{b}^{n} \\

&= (\dot{\phi})^{\wedge} \cdot \hat{C}_{b}^{n} + (I + \phi ^{\wedge}) \cdot
[\hat{C}_b^n \cdot (\hat{\omega} _{ib}^b)^{\wedge} - (\hat{\omega} _{in}^n)^{\wedge} \cdot \hat{C}_b^n] \\

\end{align} \tag{2.1.18}
$$

联合$\ref{eq:2.1.16}$和$\ref{eq:2.1.18}$：
$$
\begin{align}

(I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot (\hat{\omega} _{ib}^b + {\delta}{\omega} _{ib}^b)^{\wedge}
- (\hat{\omega} _{in}^n + {\delta}{\omega} _{in}^n)^{\wedge} \cdot (I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n}
&=
(\dot{\phi})^{\wedge} \cdot \hat{C}_{b}^{n} + (I + \phi ^{\wedge}) \cdot
[\hat{C}_b^n \cdot (\hat{\omega} _{ib}^b)^{\wedge} - (\hat{\omega} _{in}^n)^{\wedge} \cdot \hat{C}_b^n]
\\
\end{align} \tag{2.1.19}
$$

> [!NOTE]
> $$
> \begin{align}
> v_1^{\wedge} \cdot v_2^{\wedge} - v_2^{\wedge} \cdot v_1^{\wedge} &= (v_1 \times v_2)^{\wedge} \\
> C_a^b \cdot v^{\wedge} \cdot C_b^a &= (C_a^b \cdot v)^{\wedge} \\
> \end{align}
> $$


右乘$\hat{C}_{n}^{b}$：
$$
\begin{align}

(I + \phi ^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot (\hat{\omega} _{ib}^b + {\delta}{\omega} _{ib}^b)^{\wedge} \cdot \hat{C}_{n}^{b}
- (\hat{\omega} _{in}^n + {\delta}{\omega} _{in}^n)^{\wedge} \cdot (I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot \hat{C}_{n}^{b}
&=
(\dot{\phi})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot \hat{C}_{n}^{b}
+ (I + \phi ^{\wedge}) \cdot
[\hat{C}_b^n \cdot (\hat{\omega} _{ib}^b)^{\wedge} - (\hat{\omega} _{in}^n)^{\wedge} \cdot \hat{C}_b^n] \cdot \hat{C}_{n}^{b}

\\

(I + \phi ^{\wedge}) \cdot [ \hat{C}_{b}^{n} \cdot (\hat{\omega} _{ib}^b + {\delta}{\omega} _{ib}^b) ]^{\wedge}
- (\hat{\omega}_{in}^n + {\delta}{\omega} _{in}^n)^{\wedge} \cdot (I + {\phi}^{\wedge})
&=
(\dot{\phi})^{\wedge}
+ (I + \phi ^{\wedge}) \cdot [ (\hat{C}_{b}^{n} \cdot \hat{\omega} _{ib}^b)^{\wedge} - (\hat{\omega} _{in}^n)^{\wedge} ] \\

\\

(\dot{\phi})^{\wedge}
&=
(\hat{C}_{b}^{n} \cdot {\delta}{\omega}_{ib}^b)^{\wedge}
- ({\delta}{\omega} _{in}^n)^{\wedge}
+ {\phi}^{\wedge} \cdot (\hat{\omega}_{in}^n)^{\wedge}
- (\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}^{\wedge} \\
&+ {\phi}^{\wedge} \cdot (\hat{C}_{b}^{n} \cdot {\delta}{\omega} _{ib}^b)^{\wedge}
- ({\delta}{\omega} _{in}^n)^{\wedge} \cdot {\phi}^{\wedge} \quad(忽略二阶小项) \\

\\

&=
[\hat{C}_{b}^{n} \cdot (-\delta {b}_g^{b} - n_g^{b})]^{\wedge}
- ({\delta}{\omega} _{in}^n)^{\wedge}
+ ({\phi} \times \hat{\omega}_{in}^n)^{\wedge} \\

&=
[-(\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}]^{\wedge}
+ [\hat{C}_{b}^{n} \cdot (-\delta {b}_g^{b} - n_g^{b})]^{\wedge}
- ({\delta}{\omega} _{in}^n)^{\wedge} \\

&=
[
-(\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot \delta {b}_g^{b}
- \hat{C}_{b}^{n} \cdot n_g^{b}
- {\delta}{\omega} _{in}^n
]^{\wedge} \\

\\

\dot{\phi}
&=
-(\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot \delta {b}_g^{b}
- \hat{C}_{b}^{n} \cdot n_g^{b} \\
&- {\delta}{\omega} _{in}^n \quad(忽略小量)

\end{align} \tag{2.1.20}
$$

因此，**连续系统矩阵**为：
$$
\begin{align}
F_{\phi}
&=
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & -(\hat{\omega}_{in}^{n})^{\wedge} & -\hat{C}_{b}^{n} & 0_{3\times3}
\end{bmatrix}
\end{align} \tag{2.1.21}
$$

**噪声输入矩阵**为：
$$
G_{\phi}
=
\begin{bmatrix}
-\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.22}
$$


###### 完整误差传播

考虑${\delta}{\omega}_{in}^{n} = {\delta}{\omega}_{ie}^{n} + {\delta}{\omega}_{en}^{n}$，完整的姿态误差方程更新为：
$$
\begin{align}

\dot{\phi}
&=
-(\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot {\delta}{b}_g
- \hat{C}_{b}^{n} \cdot n_{g}
- ({\delta}{\omega}_{ie}^{n} + {\delta}{\omega}_{en}^{n}) \\

&=
-(\hat{\omega}_{in}^n)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot {\delta}{b}_g
- \hat{C}_{b}^{n} \cdot n_{g}
- [\hat{M}_{\omega{_{ie}}/r} \cdot \delta{r} + (\hat{M}_{\omega{_{en}}/r} \cdot \delta{r} + \hat{M}_{\omega{_{en}}/v} \cdot \delta{v^n})] \\

&=
-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) \cdot \delta{r}
-\hat{M}_{\omega{_{en}}/v} \cdot \delta{v^n}
-(\hat{\omega} _{in}^n)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot {\delta}{b}_g
- \hat{C}_{b}^{n} \cdot n_{g} \\

\end{align} \tag{2.1.23}
$$
因此，**连续系统矩阵**为：
$$
\begin{align}
F_{\phi}
&=
\begin{bmatrix}
-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) &
-\hat{M}_{\omega{_{en}}/v} &
-(\hat{\omega}_{in}^{n})^{\wedge} &
-\hat{C}_{b}^{n} &
0_{3\times3}
\end{bmatrix}
\end{align} \tag{2.1.24}
$$


**噪声输入矩阵**为：
$$
G_{\phi}
=
\begin{bmatrix}
-\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.25}
$$


##### 速度

理论导航速度为${v}^n$，计算导航速度（名义值）为$\hat{v}^n$，则速度误差为：
$$
\delta v^n = {v}^n - \widetilde{v}^n \tag{2.1.26}
$$
同时微分：
$$
\delta \dot{v}^n = \dot{v}^n - \dot{\hat{v}}^n \tag{2.1.27}
$$

真实比力方程使用**名义值 + 误差状态量**表示：
$$
\begin{align}

\dot{v}_{en}^{n}
&=
C_b^n \cdot f_{sf}^b - (2 \cdot \omega _{ie}^n + \omega _{en}^n) \times {v}_{en}^{n} + g^n \\

&=
(I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot (\hat{f}_{sf}^b + {\delta}{f}_{sf}^b) - [2 \cdot (\hat{\omega} _{ie}^n + {\delta}{\omega}_{ie}^n) + (\hat{\omega}_{en}^n + {\delta}{\omega}_{en}^n)]
\times (\hat{v}_{en}^{n} + {\delta}{v}_{en}^{n}) + (\hat{g}^{n} + {\delta}{g}^{n}) \\
\\
&=
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b + \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b + {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b \\
&- [(2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) + (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n)] \times (\hat{v}_{en}^{n} + {\delta}{v}_{en}^{n}) + (\hat{g}^{n} + {\delta}{g}^{n}) \\
\\
&=
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b + \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b \\
&- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times \hat{v}_{en}^{n}
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times {\delta}{v}_{en}^{n}
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n}
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times {\delta}{v}_{en}^{n}
+ (\hat{g}^{n} + {\delta}{g}^{n}) \\
\\

&=
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b + \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b \\
&- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times \hat{v}_{en}^{n}
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times {\delta}{v}_{en}^{n}
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n} \\
&+ (\hat{g}^{n} + {\delta}{g}^{n}) \\
&+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times {\delta}{v}_{en}^{n} \quad(忽略二阶小项)

\end{align} \tag{2.1.28}
$$

名义比力速度方程为：
$$
\begin{align}

\dot{\hat{v}}_{en}^{n}
&=
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^{b} - (2 \cdot \hat{\omega}_{ie}^{n} + \hat{\omega}_{en}^{n}) \times \hat{v}_{en}^{n} + \hat{g}^{n} \\

\end{align} \tag{2.1.29}
$$

将$\ref{eq:2.1.29}$和$\ref{eq:2.1.28}$带入$\ref{eq:2.1.27}$，得：
$$
\begin{align}

\delta \dot{v}^n
&=
[
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b + \hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times \hat{v}_{en}^{n}
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times {\delta}{v}_{en}^{n}
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n}
+ (\hat{g}^{n} + {\delta}{g}^{n})
] \\
&-
[
\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^{b} - (2 \cdot \hat{\omega}_{ie}^{n} + \hat{\omega}_{en}^{n}) \times \hat{v}_{en}^{n} + \hat{g}^{n}
] \\

&=
\hat{C}_{b}^{n} \cdot {\delta}{f}_{sf}^b
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\phi}
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n) \times {\delta}{v}_{en}^{n}
- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n}
+ {\delta}{g}^{n} \\

&=
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} \cdot {\delta}{v}_{en}^{n}
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\phi}
+ \hat{C}_{b}^{n} \cdot (-\delta {b}_a^{b} - n_a^{b}) \\
&- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n}
+ {\delta}{g}^{n} \\

&=
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} \cdot {\delta}{v}_{en}^{n}
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot \delta {b}_a^{b} - \hat{C}_{b}^{n} \cdot n_a^{b} \\
&- (2 \cdot {\delta}{\omega}_{ie}^n + {\delta}{\omega}_{en}^n) \times \hat{v}_{en}^{n}
+ {\delta}{g}^{n} \\

\end{align} \tag{2.1.30}
$$

因此，**连续系统矩阵**为：
$$
\begin{align}
F_{v}
&=
\begin{bmatrix}
0_{3\times3} & - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega} _{en}^n)^{\wedge} & - (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n}
\end{bmatrix}
\end{align} \tag{2.1.31}
$$

**噪声输入矩阵**为：
$$
G_{v}
=
\begin{bmatrix}
0_{3\times3} & -\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.32}
$$


###### 完整误差传播

考虑${\delta}{\omega}_{ie}^{n}, {\delta}{\omega}_{en}^{n}, {\delta}{g}_{}^{n}$，完整的速度误差方程更新为：
$$
\begin{align}

\delta \dot{v}^n
&=
- (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} \cdot {\delta}{v}_{en}^{n}
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot \delta {b}_a^{b} - \hat{C}_{b}^{n} \cdot n_a^{b} \\
&- [ 2 \cdot \hat{M}_{\omega{_{ie}}/r} \cdot \delta{r} + (\hat{M}_{\omega{_{en}}/r} \cdot \delta{r} + \hat{M}_{\omega{_{en}}/v} \cdot \delta{{v}_{en}^{n}}) ] \times \hat{v}_{en}^{n}
+ \hat{M}_{g/r} \cdot \delta{r} \\

\\

&=
[ (\hat{v}_{en}^{n})^{\wedge} \cdot (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r} ] \cdot \delta{r} \\
&+ [ (\hat{v}_{en}^{n})^{\wedge} \cdot \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} ] \cdot {\delta}{v}_{en}^{n} \\
&- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\phi}
- \hat{C}_{b}^{n} \cdot \delta {b}_a^{b} - \hat{C}_{b}^{n} \cdot n_a^{b} \\


\end{align} \tag{2.1.33}
$$

因此，**连续系统矩阵**为：
$$
\begin{align}
F_{v}
&=
\begin{bmatrix}
(\hat{v}_{en}^{n})^{\wedge} \cdot (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r} &
(\hat{v}_{en}^{n})^{\wedge} \cdot \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega} _{ie}^n + \hat{\omega}_{en}^n)^{\wedge} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} & 0_{3\times3} & -\hat{C}_{b}^{n}
\end{bmatrix}
\end{align} \tag{2.1.34}
$$


**噪声输入矩阵**为：
$$
G_{v}
=
\begin{bmatrix}
0_{3\times3} & -\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.35}
$$


##### 位置


###### 建模为ECEF位置误差投影到导航系


真实ECEF速度与真实导航速度的关系为：
$$
v_{}^{e} = C_{n}^{e} \cdot v_{}^{n} \tag{2.1.36}
$$
名义速度关系为：
$$
\hat{v}_{}^{e} = \hat{C}_{n}^{e} \cdot \hat{v}_{}^{n} \tag{2.1.37}
$$
则ECEF下的速度误差为：
$$
\begin{align}

{\delta}{v}_{}^{e}
&= {v}_{}^{e} - \hat{v}_{}^{e}  \\
&= C_{n}^{e} \cdot v_{}^{n} - \hat{C}_{n}^{e} \cdot \hat{v}_{}^{n} \\
&= (\hat{C}_{n}^{e} + {\delta}{C}_{n}^{e}) \cdot (\hat{v}_{}^{n} + {\delta}{v}_{}^{n}) - \hat{C}_{n}^{e} \cdot \hat{v}_{}^{n} \\
&= \hat{C}_{n}^{e} \cdot {\delta}{v}_{}^{n} + {\delta}{C}_{n}^{e} \cdot {v}_{}^{n} \\
\end{align} \tag{2.1.38}
$$

理论ECEF位置为${r}^e$，计算ECEF位置（名义值）为$\hat{r}^e$，则ECEF位置误差投影到导航系为：
$$
{\delta}r^n = \hat{C}_{e}^{n} \cdot (r_{}^{e} - \hat{r}_{}^{e}) \\ \tag{2.1.39}
$$
对时间求导：
$$
\begin{align}

{\delta}\dot{r}_{}^{n}
&= \dot{\hat{C}}_{e}^{n} \cdot (r_{}^{e} - \hat{r}_{}^{e})
+ \hat{C}_{e}^{n} \cdot ({v}_{}^{e} - \hat{v}_{}^{e}) \\

&= -({\omega}_{en}^{n})^{\wedge} \cdot \hat{C}_{e}^{n} \cdot (r_{}^{e} - \hat{r}_{}^{e})
+ \hat{C}_{e}^{n} \cdot (\hat{C}_{n}^{e} \cdot {\delta}{v}_{}^{n} + {\delta}{C}_{n}^{e} \cdot {v}_{}^{n}) \\

&= -({\omega}_{en}^{n})^{\wedge} \cdot {\delta}r^n + {\delta}{v}_{}^{n} \\
&+ \hat{C}_{e}^{n} \cdot {\delta}{C}_{n}^{e} \cdot {v}_{}^{n} \quad(忽略小量) \\

\end{align} \tag{2.1.40}
$$

连续系统矩阵**为：
$$
F_r
=
\begin{bmatrix}
-(\hat{\omega}_{en}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.41}
$$
噪声输入矩阵：
$$
\begin{align}
G_r
=
\begin{bmatrix}
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3}
\end{bmatrix}
\end{align} \tag{2.1.42}
$$



###### 建模为LLH误差

位置误差为：
$$
{\delta}r_{}^{n} =
\begin{bmatrix}
{\delta}{L} \\ {\delta}{\lambda} \\ {\delta}{h}
\end{bmatrix} \tag{2.1.43}
$$

真实位置微分方程：
$$
\begin{align}

\dot{L} &= \frac{1}{R_{M}(L) + h} \cdot v_{N} \\
\dot{\lambda} &= \frac{\sec{L}}{R_{N}(L) + h} \cdot v_{E} \\
\dot{h} &= v_{U} = -v_{D} \\

\\

\dot{\begin{bmatrix} L \\ \lambda \\ h \end{bmatrix}}
&=
\begin{bmatrix}
0 & \frac{1}{R_{M}(L) + h} & 0 \\
\frac{\sec{L}}{R_{N}(L) + h} & 0 & 0 \\
0 & 0 & 1 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
v_{E} \\ v_{N} \\ v_{U}
\end{bmatrix}

\\

&= \begin{bmatrix}
\frac{1}{R_{M}(L) + h} & 0 & 0 \\
0 & \frac{\sec{L}}{R_{N}(L) + h} & 0 \\
0 & 0 & -1 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
v_{N} \\ v_{E} \\ v_{D}
\end{bmatrix}

\end{align} \tag{2.1.44}
$$

考虑导航系为NED，名义位置微分方程：
$$
\begin{align}

\dot{\hat{L}} &= \frac{1}{{R}_{M}(\hat{L}) + \hat{h}} \cdot \hat{v}_{N} \\
\dot{\hat{\lambda}} &= \frac{\sec{\hat{L}}}{{R}_{N}(\hat{L}) + \hat{h}} \cdot \hat{v}_{E} \\
\dot{\hat{h}} &= -\hat{v}_{D} \\

\end{align} \tag{2.1.45}
$$

对位置微分方程求偏差，得**位置误差方程**为：
$$
\begin{align}

\delta \dot{L} &= - \frac{\hat{v}_{N}}{(\hat{R}_M + \hat{h})^2} \cdot {\delta}h
+ \frac{1}{\hat{R}_M + \hat{h}} \cdot {\delta}{v}_{N} \\

\delta \dot{\lambda} &=
\frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot \hat{v}_{E}}{\hat{R}_{N} + \hat{h}} \cdot {\delta}L
- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E}}{(\hat{R}_{N} + \hat{h})^2} \cdot {\delta}h
+ \frac{\sec{\hat{L}}}{\hat{R}_{N} + \hat{h}} \cdot {\delta}{v}_{E} \\

\delta \dot{h} &= \delta{v_U} = - \delta{v_D} \\

\end{align} \tag{2.1.46}
$$

**ENU:**
$$
\begin{align}

\begin{bmatrix}
\delta \dot{L} \\ \delta \dot{\lambda} \\ \delta \dot{h} \\
\end{bmatrix}
&=
\begin{bmatrix}
0 & 0 & - \frac{\hat{v}_N}{(\hat{R}_M + \hat{h})^2} \\
\frac{\hat{v}_E \cdot \sec{\hat{L}} \cdot \tan{\hat{L}}}{\hat{R}_N + \hat{h}} & 0 & - \frac{\hat{v}_E \cdot \sec{\hat{L}}}{(\hat{R}_N + \hat{h})^2} \\
0 & 0 & 0 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
\delta{L} \\ \delta{\lambda} \\ \delta{h} \\
\end{bmatrix}

+

\begin{bmatrix}
0 & \frac{1}{\hat{R}_M + \hat{h}} & 0 \\
\frac{\sec{\hat{L}}}{\hat{R}_N + \hat{h}} & 0 & 0 \\
0 & 0 & 1 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
\delta{v_E} \\ \delta{v_N} \\ \delta{v_U} \\
\end{bmatrix}

\\

\delta \dot{r} &= \hat{M}_{r/r} \cdot \delta{r} + \hat{M}_{r/v} \cdot \delta{v_{ENU}^n} \\

\end{align} \tag{2.1.47}
$$

**NED:**
$$
\begin{align}

\begin{bmatrix}
\delta \dot{L} \\ \delta \dot{\lambda} \\ \delta \dot{h} \\
\end{bmatrix}
&=
\begin{bmatrix}
0 & 0 & - \frac{\hat{v}_N}{(\hat{R}_M + \hat{h})^2} \\
\frac{\hat{v}_E \cdot \sec{\hat{L}} \cdot \tan{\hat{L}}}{\hat{R}_N + \hat{h}} & 0 & - \frac{\hat{v}_E \cdot \sec{\hat{L}}}{(\hat{R}_N + \hat{h})^2} \\
0 & 0 & 0 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
\delta{L} \\ \delta{\lambda} \\ \delta{h} \\
\end{bmatrix}

+

\begin{bmatrix}
\frac{1}{\hat{R}_M + \hat{h}} & 0 & 0 \\
0 & \frac{\sec{\hat{L}}}{\hat{R}_N + \hat{h}} & 0 \\
0 & 0 & -1 \\
\end{bmatrix}

\cdot

\begin{bmatrix}
\delta{v_N} \\ \delta{v_E} \\ \delta{v_D} \\
\end{bmatrix}

\\

\delta \dot{r} &= \hat{M}_{r/r} \cdot \delta{r} + \hat{M}_{r/v} \cdot \delta{v_{NED}^n} \\

\end{align} \tag{2.1.48}
$$

**连续系统矩阵**为：
$$
F_r
=
\begin{bmatrix}
\hat{M}_{r/r} & \hat{M}_{r/v} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{2.1.49}
$$
噪声输入矩阵：
$$
\begin{align}
G_r
=
\begin{bmatrix}
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3}
\end{bmatrix}
\end{align} \tag{2.1.50}
$$


-  **纬度**

$R_M$是关于$L$的函数，将$R_M(L)$在计算导航纬度（名义值）$\hat{L}$出一阶泰勒展开：
$$
\begin{align}

R_M(L)
&= R_M(\hat{L} + {\delta}L) \\
&\approx R_M(\hat{L}) + \frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L \\

\end{align} \tag{2.1.51}
$$
则有：
$$
\begin{align}

R_M(L) + h
&\approx
[R_M(\hat{L}) + \frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L] + [\hat{h} + {\delta}h] \\

&=
[R_M(\hat{L}) + \hat{h}]
+ [\frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L
+ {\delta}h] \\

&=
\hat{R}_{Mh} + \epsilon \\

\end{align} \tag{2.1.52}
$$

> [!分母展开]
> 原理：当$\vert \frac{\epsilon}{\hat{R}_{Mh}} \vert \lll 1$时，利用一阶泰勒展开：
> $$
> \begin{align}
> \frac{1}{ \hat{R}_{Mh} + \epsilon }
> = \frac{1}{ \hat{R}_{Mh} } \cdot \frac{1}{ 1 + \frac{ \epsilon }{ \hat{R}_{Mh} } }
> \approx \frac{1}{\hat{R}_{Mh}} \cdot ( 1 - \frac{ \epsilon }{ \hat{R}_{Mh} } )
> \end{align}
> $$
> 

真实纬度微分方程更新为：
$$
\begin{align}

\dot{L}
&=
\frac{\hat{v}_{N} + {\delta}{v}_{N}}{\hat{R}_{Mh} + \epsilon} \\

&\approx
\frac{\hat{v}_{N} + {\delta}{v}_{N}}{\hat{R}_{Mh}} \cdot
(1 - \frac{\epsilon}{\hat{R}_{Mh}}) \\

&\approx
\frac{\hat{v}_{N} + {\delta}{v}_{N}}{\hat{R}_{Mh}}
- \frac{(\hat{v}_{N} + {\delta}{v}_{N}) \cdot \epsilon}{(\hat{R}_{Mh})^2} \\
\\
&\approx
\frac{\hat{v}_{N}}{\hat{R}_{Mh}}
+ \frac{{\delta}{v}_{N}}{\hat{R}_{Mh}}
- \frac{\hat{v}_{N} \cdot \epsilon}{(\hat{R}_{Mh})^2} \\
&- \frac{{\delta}{v}_{N} \cdot \epsilon}{(\hat{R}_{Mh})^2} \quad (忽略二阶误差项)
\\
\\
&\approx
\frac{\hat{v}_{N}}{\hat{R}_{Mh}} \quad(名义主项)
\\
&+ \frac{{\delta}{v}_{N}}{\hat{R}_{Mh}} \quad(速度误差项)
\\
&- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot \frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L \quad(曲率半径随纬度的变化项)
\\
&- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot {\delta}h \quad(高度误差项)
\\

\end{align} \tag{2.1.53}
$$


名义纬度微分方程为：
$$
\begin{align}
\dot{\hat{L}}
&= \frac{1}{{R}_{M}(\hat{L}) + \hat{h}} \cdot \hat{v}_{N} \\
&= \frac{\hat{v}_{N}}{\hat{R}_{Mh}} \\
\end{align} \tag{2.1.54}
$$

纬度误差方程：
$$
\begin{align}

{\delta}\dot{L} = \dot{L} - \dot{\hat{L}}
&=
[\frac{\hat{v}_{N}}{\hat{R}_{Mh}}
+ \frac{{\delta}{v}_{N}}{\hat{R}_{Mh}}
- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot \frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L
- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot {\delta}h ]
-
\frac{\hat{v}_{N}}{\hat{R}_{Mh}} \\

&=
\frac{{\delta}{v}_{N}}{\hat{R}_{Mh}}
- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot {\delta}h \\
&- \frac{\hat{v}_{N}}{(\hat{R}_{Mh})^2} \cdot \frac{\partial R_M}{\partial L} |_{\hat{L}} \cdot {\delta}L \quad(忽略地球曲率半径随纬度的变化) \\

&\approx
- \frac{\hat{v}_{N}}{(\hat{R}_M + \hat{h})^2} \cdot {\delta}h
+ \frac{1}{\hat{R}_M + \hat{h}} \cdot {\delta}{v}_{N} \\

\end{align} \tag{2.1.55}
$$

- **经度**

$\sec{L}$是关于$L$的函数，在计算导航纬度（名义值）$\hat{L}$出一阶泰勒展开：
$$
\begin{align}

\sec{L} = \sec{(\hat{L} + {\delta}L)}
&\approx \sec{\hat{L}} + \frac{\mathbf{d} \sec{L}}{\mathbf{d} L} |_{\hat{L}} \cdot {\delta}L \\

&\approx \sec{\hat{L}} + \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \\

\end{align} \tag{2.1.56}
$$
$R_N$是关于$L$的函数，将$R_N(L)$在计算导航纬度（名义值）$\hat{L}$出一阶泰勒展开：
$$
\begin{align}

R_N(L)
&= R_N(\hat{L} + {\delta}L) \\
&\approx R_N(\hat{L}) + \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L \\

\end{align} \tag{2.1.57}
$$
则有：
$$
\begin{align}

R_N(L) + h
&\approx
[R_N(\hat{L}) + \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L] + [\hat{h} + {\delta}h] \\

&=
[R_N(\hat{L}) + \hat{h}]
+ [ \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L + {\delta}h ] \\

&=
\hat{R}_{Nh} + \epsilon \\

\end{align} \tag{2.1.58}
$$

真实经度微分方程更新为：
$$
\begin{align}

\dot{\lambda}
&= \frac{1}{\hat{R}_{Nh} + \epsilon} \cdot
(\sec{\hat{L}} + \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L) \cdot (\hat{v}_{E} + {\delta}{v}_{E}) \\
\\
&\approx
\frac{1}{\hat{R}_{Nh}} \cdot (1 - \frac{\epsilon}{\hat{R}_{Nh}}) \cdot \\
&[
\sec{\hat{L}} \cdot \hat{v}_{E} + \sec{\hat{L}} \cdot {\delta}{v}_{E}
+ \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E} \\
&+ \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot {\delta}{v}_{E}
] \quad (忽略二阶小量) \\
\\
&\approx
\frac{ \sec{\hat{L}} \cdot \hat{v}_{E} + \sec{\hat{L}} \cdot {\delta}{v}_{E}
+ \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E} }{\hat{R}_{Nh}}
- \frac{ ( \sec{\hat{L}} \cdot \hat{v}_{E} + \sec{\hat{L}} \cdot {\delta}{v}_{E}
+ \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}) \cdot \epsilon }{ (\hat{R}_{Nh})^2 } \\
\\
&\approx
\frac{\sec{\hat{L}} \cdot \hat{v}_{E}}{\hat{R}_{Nh}}
+ \frac{\sec{\hat{L}} \cdot {\delta}{v}_{E}}{\hat{R}_{Nh}}
+ \frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}}{\hat{R}_{Nh}}

- \frac{\sec{\hat{L}} \cdot \hat{v}_{E} \cdot ( \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L + {\delta}h ) }{(\hat{R}_{Nh})^2} \\
&- \frac{ (\sec{\hat{L}} \cdot {\delta}{v}_{E} + \sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}) \cdot \epsilon }{(\hat{R}_{Nh})^2} \quad (忽略二阶小量) \\
\\
&\approx
\frac{\sec{\hat{L}} \cdot \hat{v}_{E}}{\hat{R}_{Nh}} \quad(名义主项) \\
&+ \frac{\sec{\hat{L}} \cdot {\delta}{v}_{E}}{\hat{R}_{Nh}} \quad(速度误差项) \\
&+ \frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}}{\hat{R}_{Nh}} \quad(\sec{L}随纬度的变化项) \\
&- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L }{(\hat{R}_{Nh})^2} \quad(R_N随纬度的变化项) \\
&- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot {\delta}h }{(\hat{R}_{Nh})^2} \quad(高度误差项) \\

\end{align} \tag{2.1.59}
$$

名义经度微分方程为：
$$
\begin{align}

\dot{\hat{\lambda}}
&=
\frac{\sec{\hat{L}}}{{R}_{N}(\hat{L}) + \hat{h}} \cdot \hat{v}_{E} \\

&=
\frac{\sec{\hat{L}} \cdot \hat{v}_{E}}{ \hat{R}_{Nh} } \\

\end{align} \tag{2.1.60}
$$


经度误差方程：
$$
\begin{align}
{\delta}{\dot{\lambda}} &= \dot{\lambda} - \dot{\hat{\lambda}} \\

&=
[
\frac{\sec{\hat{L}} \cdot \hat{v}_{E}}{\hat{R}_{Nh}}
+ \frac{\sec{\hat{L}} \cdot {\delta}{v}_{E}}{\hat{R}_{Nh}}
+ \frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}}{\hat{R}_{Nh}}
- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L }{(\hat{R}_{Nh})^2}
- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot {\delta}h }{(\hat{R}_{Nh})^2}
]
-
\frac{\sec{\hat{L}} \cdot \hat{v}_{E}}{ \hat{R}_{Nh} } \\

&=
\frac{\sec{\hat{L}} \cdot {\delta}{v}_{E}}{\hat{R}_{Nh}}
+ \frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot {\delta}L \cdot \hat{v}_{E}}{\hat{R}_{Nh}}
- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot {\delta}h }{(\hat{R}_{Nh})^2} \\
&- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E} \cdot \frac{\partial R_N}{\partial L} |_{\hat{L}} \cdot {\delta}L }{(\hat{R}_{Nh})^2} \quad(忽略地球曲率半径随纬度的变化) \\

&=
\frac{\sec{\hat{L}} \cdot \tan{\hat{L}} \cdot \hat{v}_{E}}{\hat{R}_{N} + \hat{h}} \cdot {\delta}L
- \frac{ \sec{\hat{L}} \cdot \hat{v}_{E}}{(\hat{R}_{N} + \hat{h})^2} \cdot {\delta}h
+ \frac{\sec{\hat{L}}}{\hat{R}_{N} + \hat{h}} \cdot {\delta}{v}_{E} \\

\end{align} \tag{2.1.61}
$$

- **高程**

高程误差方程为：
$$
\begin{align}
{\delta}{\dot{h}} &= \dot{h} - \dot{\hat{h}} \\
&= (-v_{D}) - (- \hat{v}_D) \\
&= - (v_{D} - \hat{v}_D) \\
&= -{\delta}v_{D} \\
\end{align} \tag{2.1.62}
$$




### 2.1.2 连续时间过程噪声的功率谱密度矩阵: Qc

- 位置状态不直接受传感器噪声驱动

- **零偏建模为随机游走**
$$
Q_c =
\begin{bmatrix}
{{\sigma}_{g}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & {{\sigma}_{a}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & {{\sigma}_{b_g}^{rw}}^{2} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & {{\sigma}_{b_a}^{rw}}^{2} \cdot I_{3\times3} \\
\end{bmatrix} \tag{2.1.63}
$$

- **零偏建模为FOGM**
$$
Q_c =
\begin{bmatrix}
{{\sigma}_{g}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & {{\sigma}_{a}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & \frac{2 \cdot {\sigma_{b_g}^{ss}}^{2}}{\tau _c} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & \frac{2 \cdot {\sigma_{b_a}^{ss}}^{2}}{\tau _c} \cdot I_{3\times3} \\
\end{bmatrix} \tag{2.1.64}
$$


### 2.1.3 噪声输入矩阵: G
$$
G
=
\begin{bmatrix}
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} \\
0_{3\times3} & -\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3} \\
-\hat{C}_{b}^{n} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times 3} & 0_{3\times 3} & I_{3\times 3} & 0_{3\times 3} \\
0_{3\times 3} & 0_{3\times 3} & 0_{3\times 3} & I_{3\times 3} \\
\end{bmatrix} \tag{2.1.65}
$$


# 3. 离散模型


## 3.1 离散误差状态模型（预测）


> [!预测公式]
> $$
> \begin{align}
> {\delta} \hat{X}_{k|k-1} &= {\Phi}_{k-1} \cdot {\delta} \hat{X}_{k-1|k-1} \\
> \\
> P_{k|k-1} &= {\Phi}_{k-1} \cdot P_{k-1|k-1} \cdot {\Phi}_{k-1}^{T} + Q_{d} \\
> &= {\Phi}_{k-1} \cdot P_{k-1|k-1} \cdot {\Phi}_{k-1}^{T} + \frac{\Delta t}{2} \cdot
> [
> {\Phi}_{k-1} \cdot (G_{k-1} \cdot Q_{c,k-1} \cdot G_{k-1}^T) + (G_{k-1} \cdot Q_{c,k-1} \cdot G_{k-1}^T) \cdot {\Phi}_{k-1}^{T}
> ]
> \end{align}
> $$
> 


在采样周期$\Delta t = t_k - t_{k-1}$内，若$F$近似为常值，则$\ref{eq:2.1.2}$可以精确离散化为：
$$
\begin{align}

{\delta}x_{k+1} = e^{F \cdot \Delta t} \cdot {\delta}x_{k} + {\int}_{0}^{\Delta t} e^{F \cdot (\Delta t - \tau)} \cdot G \cdot {\omega}_{c}(t_k + \tau) \mathbf{d} \tau \\

\end{align} \tag{3.1.1}
$$
令
$$
\Phi = e^{F \cdot \Delta t} \tag{3.1.2}
$$
当$\Delta t$较小时，可以对指数矩阵进行一阶截断：
$$
\begin{align}
e^{F \cdot \Delta t} &= {\sum}_{k=0}^{\infty}{\frac{(F \cdot \Delta t)^{k}}{k!}} \\
&\approx I + F \cdot \Delta t \\
e^{F \cdot (\Delta t - \tau)} &\approx I + F \cdot (\Delta t - \tau) \\
\end{align} \tag{3.1.3}
$$

则有：
$$
\begin{align}

{\delta}x_{k+1} &= \Phi \cdot {\delta}x_{k} + {\int}_{0}^{\Delta t} e^{F \cdot (\Delta t - \tau)} \cdot G \cdot {\omega}_{c}(t_k + \tau) \mathbf{d} \tau \\

&= \Phi \cdot {\delta}x_{k} + {\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \tau)] \cdot G \cdot {\omega}_{c}(t_k + \tau) \mathbf{d} \tau \\

&= \Phi \cdot {\delta}x_{k} + {\omega}_k \\

\end{align} \tag{3.1.4}
$$

### 3.1.1 离散状态转移矩阵: $\Phi$


- **零偏建模为随机游走**

完整的==工程==离散状态转移矩阵$\Phi$为：
$$
\Phi
=
\begin{bmatrix}

-({\omega}_{en}^{n})^{\wedge} \cdot \Delta t + I_{3\times3}  & I_{3\times3} \cdot \Delta t & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

[\hat{v}_{en}^{n} \times (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r}] \cdot \Delta t &
[\hat{v}_{en}^{n} \times \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega}_{ie}^n + \hat{\omega}_{en}^n)^{\wedge}] \cdot \Delta t + I_{3\times3} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot \Delta t &
0_{3\times3} &
-\hat{C}_{b}^{n} \cdot \Delta t \\

-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) \cdot \Delta t &
-\hat{M}_{\omega{_{en}}/v} \cdot \Delta t &
-(\hat{\omega}_{in}^{n})^{\wedge} \cdot \Delta t + I_{3\times3} &
-\hat{C}_{b}^{n} \cdot \Delta t &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} \\

\end{bmatrix} \tag{3.1.5}
$$

近似==工程==离散状态转移矩阵$\Phi$为：
$$
\Phi
=
\begin{bmatrix}

-({\omega}_{en}^{n})^{\wedge} \cdot \Delta t + I_{3\times3}  & I_{3\times3} \cdot \Delta t & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

0_{3\times3} &
- (2 \cdot \hat{\omega}_{ie}^n + \hat{\omega}_{en}^n)^{\wedge} \cdot {\Delta}t + I_{3\times3} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\Delta}t &
0_{3\times3} &
-\hat{C}_{b}^{n} \cdot {\Delta}t \\

0_{3\times3} &
0_{3\times3} &
-(\hat{\omega}_{in}^{n})^{\wedge} \cdot {\Delta}t + I_{3\times3} &
-\hat{C}_{b}^{n} \cdot {\Delta}t &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} \\

\end{bmatrix} \tag{3.1.6}
$$


- **零偏建模为FOGM**

完整的==工程==离散状态转移矩阵$\Phi$为：
$$
\Phi
=
\begin{bmatrix}

-({\omega}_{en}^{n})^{\wedge} \cdot \Delta t + I_{3\times3}  & I_{3\times3} \cdot \Delta t & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

[\hat{v}_{en}^{n} \times (2 \cdot \hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) + \hat{M}_{g/r}] \cdot \Delta t &
[\hat{v}_{en}^{n} \times \hat{M}_{\omega{_{en}}/v} - (2 \cdot \hat{\omega}_{ie}^n + \hat{\omega}_{en}^n)^{\wedge}] \cdot \Delta t + I_{3\times3} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot \Delta t &
0_{3\times3} &
-\hat{C}_{b}^{n} \cdot \Delta t \\

-(\hat{M}_{\omega{_{ie}}/r} + \hat{M}_{\omega{_{en}}/r}) \cdot \Delta t &
-\hat{M}_{\omega{_{en}}/v} \cdot \Delta t &
-(\hat{\omega}_{in}^{n})^{\wedge} \cdot \Delta t + I_{3\times3} &
-\hat{C}_{b}^{n} \cdot \Delta t &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & e^{- \frac{\Delta t}{\tau{_{g,c}}}} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & e^{- \frac{\Delta t}{\tau{_{a,c}}}} \cdot I_{3\times3} \\


\end{bmatrix} \tag{3.1.7}
$$

近似==工程==离散状态转移矩阵$\Phi$为：
$$
\Phi
=
\begin{bmatrix}

-({\omega}_{en}^{n})^{\wedge} \cdot \Delta t + I_{3\times3}  & I_{3\times3} \cdot \Delta t & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\

0_{3\times3} &
- (2 \cdot \hat{\omega}_{ie}^n + \hat{\omega}_{en}^n)^{\wedge} \cdot {\Delta}t + I_{3\times3} &
- (\hat{C}_{b}^{n} \cdot \hat{f}_{sf}^b)^{\wedge} \cdot {\Delta}t &
0_{3\times3} &
-\hat{C}_{b}^{n} \cdot {\Delta}t \\

0_{3\times3} &
0_{3\times3} &
-(\hat{\omega}_{in}^{n})^{\wedge} \cdot {\Delta}t + I_{3\times3} &
-\hat{C}_{b}^{n} \cdot {\Delta}t &
0_{3\times3} \\

0_{3\times3} & 0_{3\times3} & 0_{3\times3} & e^{- \frac{\Delta t}{\tau{_{g,c}}}} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & e^{- \frac{\Delta t}{\tau{_{a,c}}}} \cdot I_{3\times3} \\

\end{bmatrix} \tag{3.1.8}
$$


### 3.1.2 离散过程噪声: Qd

> [!$\delta$函数]
> $\delta$函数为偶函数；
> 
> 筛选性质：
> $$
> \begin{align}
> {\int}_{-\infty}^{+\infty} f(x) \cdot \delta(x - a) \mathbf{d}x &= f(a) \\
> {\int}_{-\infty}^{+\infty} f(x) \cdot \delta(a - x) \mathbf{d}x &= {\int}_{-\infty}^{+\infty} f(x) \cdot \delta(-(x - a)) \mathbf{d}x \\
> &= {\int}_{-\infty}^{+\infty} f(x) \cdot \delta(x - a) \mathbf{d}x \\
> &= f(a) \\
> \end{align}
> $$


**注：**

- 噪声均值
$$
\begin{align}

\mathbf{E}[ {\omega}_{c}(t_k + \tau) \cdot  {\omega}_{c}^{T}(t_k + \sigma)]
&=
Q_{c} \cdot \delta [(t_k + \tau) - (t_k + \sigma)] \\

&=
Q_{c} \cdot \delta (\tau - \sigma) \\

\end{align} \tag{3.1.9}
$$

- 二重积分简化为一重积分（先对$\sigma$积分）
	- $\tau \in [0, \Delta t]$
$$
\begin{align}

{\int}_{0}^{\Delta t}
Q_{c} \cdot \delta (\tau - \sigma) \cdot \{ G^T \cdot [I + F \cdot (\Delta t - \sigma)]^T \}
\mathbf{d}\sigma

&=
Q_{c} \cdot {\int}_{0}^{\Delta t}
\delta (\tau - \sigma) \cdot f(\sigma)
\mathbf{d}\sigma \\

&=
Q_{c} \cdot f(\tau) \\

&=
Q_{c} \cdot G^T \cdot [I + F \cdot (\Delta t - \tau)]^T \\

\end{align} \tag{3.1.10}
$$


对于离散过程噪声$\omega _k$，对应的协方差矩阵$Q_d$为：
$$
\begin{align}
Q_d
&= \mathbf{E}[\omega _k \cdot \omega _k^T] \\

&= \mathbf{E} [
({\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \tau)] \cdot G \cdot {\omega}_{c}(t_k + \tau) \mathbf{d} \tau)
\cdot
({\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \sigma)] \cdot G \cdot {\omega}_{c}(t_k + \sigma) \mathbf{d} \sigma)^T
] \\

&= \mathbf{E} [
({\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \tau)] \cdot G \cdot {\omega}_{c}(t_k + \tau) \mathbf{d} \tau)
\cdot
({\int}_{0}^{\Delta t} {\omega}_{c}^{T}(t_k + \sigma) \cdot G^T \cdot [I + F \cdot (\Delta t - \sigma)]^T \mathbf{d} \sigma)
] \\

&=
{\int}_{0}^{\Delta t} {\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \tau)] \cdot G \cdot
\mathbf{E}[ {\omega}_{c}(t_k + \tau) \cdot  {\omega}_{c}^{T}(t_k + \sigma)] \cdot
G^T \cdot [I + F \cdot (\Delta t - \sigma)]^T
\mathbf{d} \tau \mathbf{d} \sigma \\

&=
{\int}_{0}^{\Delta t} {\int}_{0}^{\Delta t} [I + F \cdot (\Delta t - \tau)] \cdot G \cdot
[Q_{c} \cdot \delta(\tau - \sigma)] \cdot
G^T \cdot [I + F \cdot (\Delta t - \sigma)]^T
\mathbf{d} \tau \mathbf{d} \sigma \\

&=
{\int}_{0}^{\Delta t}
[I + F \cdot (\Delta t - \tau)] \cdot
G \cdot Q_{c} \cdot G^T \cdot
[I + F \cdot (\Delta t - \tau)]^T
\mathbf{d}\tau \\

&=
{\int}_{0}^{\Delta t}
(G \cdot Q_{c} \cdot G^T) +
F \cdot (G \cdot Q_{c} \cdot G^T) \cdot (\Delta t - \tau) +
(G \cdot Q_{c} \cdot G^T) \cdot F^T \cdot (\Delta t - \tau) +
F \cdot (G \cdot Q_{c} \cdot G^T) \cdot F^T \cdot (\Delta t - \tau)^{2}
\mathbf{d}\tau \\

&=
(G \cdot Q_{c} \cdot G^T) \cdot \Delta t
+ \frac{\Delta t^2}{2} \cdot [F \cdot (G \cdot Q_{c} \cdot G^T) + (G \cdot Q_{c} \cdot G^T) \cdot F^T] \\
&+ \frac{\Delta t^3}{3} \cdot [F \cdot (G \cdot Q_{c} \cdot G^T) \cdot F^T] \quad(工程上，忽略三阶项) \\

\\
&---------------------------------------- \\

&=
\frac{\Delta t}{2} \cdot
\{ 2 \cdot (G \cdot Q_{c} \cdot G^T) + \Delta t \cdot [F \cdot (G \cdot Q_{c} \cdot G^T) + (G \cdot Q_{c} \cdot G^T) \cdot F^T] \} \\

&=
\frac{\Delta t}{2} \cdot
\{
(G \cdot Q_{c} \cdot G^T) + \Delta t \cdot F \cdot (G \cdot Q_{c} \cdot G^T) +
(G \cdot Q_{c} \cdot G^T) + \Delta t \cdot (G \cdot Q_{c} \cdot G^T) \cdot F^T
\} \\

&=
\frac{\Delta t}{2} \cdot
\{
(I + F \cdot \Delta t) \cdot (G \cdot Q_{c} \cdot G^T) +
(G \cdot Q_{c} \cdot G^T) \cdot (I + F \cdot \Delta t)^T \\

&=
\frac{\Delta t}{2} \cdot
[
\Phi \cdot (G \cdot Q_{c} \cdot G^T) + (G \cdot Q_{c} \cdot G^T) \cdot {\Phi}^{T}
] \\

\end{align} \tag{3.1.11}
$$


- **零偏建模为随机游走**
$$
Q = G \cdot Q_c \cdot G^T =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & {{\sigma}_{g}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & {{\sigma}_{a}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & {{\sigma}_{b_g}^{rw}}^{2} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & {{\sigma}_{b_a}^{rw}}^{2} \cdot I_{3\times3} \\
\end{bmatrix} \tag{3.1.12}
$$

- **零偏建模为FOGM**
$$
Q = G \cdot Q_c \cdot G^T =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & {{\sigma}_{g}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & {{\sigma}_{a}^{2}} \cdot I_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & \frac{2 \cdot {\sigma_{b_g}^{ss}}^{2}}{\tau _c} \cdot I_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & \frac{2 \cdot {\sigma_{b_a}^{ss}}^{2}}{\tau _c} \cdot I_{3\times3} \\
\end{bmatrix} \tag{3.1.13}
$$

==前提条件==：
- 姿态矩阵为正交
	- $C \cdot C^T = I$
	- 旋转不改变各向同性噪声的统计特性，b系下的噪声投影到n系后，噪声协方差仍保持各向同性且方差不变
- 噪声各向异性
	- b系下的噪声在各个方向上的统计特性相同（三个轴方差相等且不相关）
- 忽略IMU三轴的安装误差和尺度因子




## 3.2 离散观测模型（更新）


> [!更新公式]
> $$
> \begin{align}
> K_k &= P_{k|k-1} \cdot H_k^T \cdot (H_k \cdot P_{k|k-1} \cdot H_k^T + R_k)^{-1} \\
> {\delta}\hat{X}_{k|k} &= K_k \cdot [Z_k - h(\hat{X}_{k|k-1})] \\
> P_{k|k} &= (I - K_k \cdot H_k) \cdot P_{k|k-1} \\
> \end{align}
> $$
> 

> [!预测完成后的误差状态]
> 预测完成后，误差状态的先验分布为：
> $$
> \begin{align}
{\delta}X_{k|k-1} &\sim \mathbf{N}({\delta}\hat{X}_{k|k-1}, P_{k|k-1}) \\
&\sim \mathbf{N}(0, P_{k|k-1}) \\
\end{align}
> $$
> 误差状态的先验均值${\delta}\hat{X}_{k|k-1}$变为0的原因：
> - 上一个时刻反馈校正后，执行${\delta}\hat{X}_{k-1|k-1} = 0$，在预测步有：${\delta}\hat{X}_{k|k-1} = {\Phi}_{k-1} \cdot {\delta}\hat{X}_{k-1|k-1} = 0$
> - 预测阶段产生的误差估计添加到名义状态；
> - 误差状态为小量，置0保持线性化点始终接近真实状态；
> - 避免线性化失效；


根据ESKF约定，有：
$$
\begin{align}
X_k &= \hat{X}_{k|k-1} \oplus \delta X_{k|k-1} \\
\delta X_{k|k-1} &\sim \mathbf{N}({\delta}\hat{X}_{k|k-1}, P_{k|k-1}) \\
&\sim \mathbf{N}(0, P_{k|k-1}) \\
\end{align} \tag{3.2.1}
$$

则观测方程为：
$$
\begin{align}
Z_k &= h(X_k) + {\nu}_{k} \\
&= h(\hat{X}_{k|k-1} \oplus {\delta}X_{k|k-1}) + {\nu}_{k} ,\quad {\nu}_{k} \sim \mathbf{N}(0, R_{k}) \\
\end{align} \tag{3.2.2}
$$

在误差状态先验均值${\delta}X_{k|k-1} = {\delta}\hat{X}_{k|k-1} = 0$处（简化为在名义状态处展开）进行一阶泰勒展开：
$$
\begin{align}

h(\hat{X}_{k|k-1} \oplus {\delta}X_{k|k-1})

&\approx
h(\hat{X}_{k|k-1} \oplus {\delta}\hat{X}_{k|k-1}) + H_{k} \cdot [{\delta}X_{k|k-1} - {\delta}\hat{X}_{k|k-1}]
,\quad H_{k} = \frac{\partial h(\hat{X}_{k|k-1} \oplus {\delta}X_{k|k-1})}{\partial {\delta}X_{k|k-1}}|_{{\delta}X_{k|k-1} = {\delta}\hat{X}_{k|k-1}} \\

&\approx
h(\hat{X}_{k|k-1}) + H_{k} \cdot {\delta}X_{k|k-1}
,\quad H_{k} = \frac{\partial h(\hat{X}_{k|k-1} \oplus {\delta}X_{k|k-1})}{\partial {\delta}X_{k|k-1}}|_{{\delta}X_{k|k-1} = 0} \\

\end{align} \tag{3.2.3}
$$

则**离散观测方程**为：
$$
\begin{align}
Z_k
&\approx h(\hat{X}_{k|k-1}) + H_{k} \cdot {\delta}X_{k|k-1} + {\nu}_{k} \\
\end{align} \tag{3.2.4}
$$


**观测残差**（新息）定义为==观测 - 预测==：
$$
\begin{align}
r_k &= Z_k - h(\hat{X}_{k|k-1}) \\
&= [h(\hat{X}_{k|k-1}) + H_{k} \cdot {\delta}X_{k|k-1} + {\nu}_{k}] - h(\hat{X}_{k|k-1}) \\
&= H_{k} \cdot {\delta}X_{k|k-1} + {\nu}_{k} \\
\end{align} \tag{3.2.5}
$$

由${\delta}X_{k|k-1} \sim \mathbf{N}(0, P_{k|k-1})$和${\nu}_{k} \sim \mathbf{N}(0, R_{k})$，且相互独立，得**新息的协方差矩阵**为：
$$
S_k = H_k \cdot P_{k|k-1} \cdot H_k^T + R_k \\ \tag{3.2.6}
$$

**互协方差矩阵**为：
$$
\mathbf{Cov}({\delta}X_{k|k-1}, r_{k}) = P_{k|k-1} \cdot H_k^T \\ \tag{3.2.7}
$$

**联合高斯分布**为：
$$
\begin{align}

\begin{bmatrix}
{\delta}X_{k|k-1} \\ r_{k} \\
\end{bmatrix}
\sim
\mathbf{N}
(
\begin{bmatrix}
0 \\ 0 \\
\end{bmatrix}
,\quad
\begin{bmatrix}
P_{k|k-1} & P_{k|k-1} \cdot H_k^T \\
H_k \cdot P_{k|k-1} & S_k \\
\end{bmatrix}
)
\\
\end{align} \tag{3.2.8}
$$

在给定新息$r_k$的条件下，**误差状态后验的均值**${\delta}\hat{X}_{k|k}$为：
$$
\begin{align}
{\delta}\hat{X}_{k|k} &= \mathbf{E}[{\delta}X_{k|k-1} | r_k] \\
&= P_{k|k-1} \cdot H_k^T \cdot S_k^{-1} \cdot r_k \\
&= P_{k|k-1} \cdot H_k^T \cdot (H_k \cdot P_{k|k-1} \cdot H_k^T + R_k)^{-1} \cdot r_k \\
&= K_k \cdot r_k \\
\end{align} \tag{3.2.9}
$$
其中，令**增益**为：
$$
K_k = P_{k|k-1} \cdot H_k^T \cdot (H_k \cdot P_{k|k-1} \cdot H_k^T + R_k)^{-1} \\ \tag{3.2.10}
$$


综上，**后验误差状态估计**为：
$$
\begin{align}
{\delta}\hat{X}_{k|k} &= {\delta}\hat{X}_{k|k-1} + K_k \cdot r_k \\
&= 0 + K_k \cdot [Z_k - h(\hat{X}_{k|k-1})] \\
&= K_k \cdot [Z_k - h(\hat{X}_{k|k-1})] \\
\end{align} \tag{3.2.11}
$$

**条件后验协方差**：
$$
\begin{align}
P_{k|k} &= P_{k|k-1} - P_{k|k-1} \cdot H_k^T \cdot S_k^{-1} \cdot H_k \cdot P_{k|k-1} \\
&= P_{k|k-1} - P_{k|k-1} \cdot H_k^T \cdot [H_k \cdot P_{k|k-1} \cdot H_k^T + R_k]^{-1} \cdot H_k \cdot P_{k|k-1} \\
&= P_{k|k-1} - K_k \cdot H_k \cdot P_{k|k-1} \\
&= (I - K_k \cdot H_k) \cdot P_{k|k-1} \\
\end{align} \tag{3.2.12}
$$

缺点：
- 在数值计算时因舍入误差导致$P_{k|k}$失去**对称性/正定性**

理论策略：采用Joseph形式：
$$
P_{k|k} = (I - K_k \cdot H_k) \cdot P_{k|k-1} \cdot (I - K_k \cdot H_k)^{T} + K_k \cdot R_K \cdot K_k^{T} \\ \tag{3.2.13}
$$

### 3.2.1 观测矩阵的构建

- 统一采用==真实值 = 名义值 + 误差状态量==


#### GNSS

约定：
- 导航系为NED

##### Loose-Couping
###### Position

在ECEF系下建立GNSS天线相位中心与IMU测量中心的位置关系，

真实位置使用**名义值 + 误差状态量**表示为：
$$
\begin{align}

r_G^e
&=
r_I^e + C_{n}^e \cdot C_b^n \cdot l^b \\
&=
(\hat{r}_I^e + {\delta}r_I^e) +
( \hat{C}_{n}^{e} + \frac{\partial {C}_{n}^{e}}{\partial L}|_{\hat{L}} \cdot {\delta}L + \frac{\partial {C}_{n}^{e}}{\partial {\lambda}}|_{\hat{\lambda}} \cdot {\delta}{\lambda} ) \cdot
[ (I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^{n} ] \cdot l^b \\

&=
(\hat{r}_I^e + {\delta}r_I^e) +
[
\hat{C}_{n}^{e} \cdot \hat{C}_{b}^{n}
+ \hat{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n}
+ {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n}
+ {\delta}{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n}
] \cdot l^b \\

\end{align} \tag{3.2.14}
$$
其中，令：
$$
{\delta}{C}_{n}^{e} =
\frac{\partial {C}_{n}^{e}}{\partial L}|_{\hat{L}} \cdot {\delta}L + \frac{\partial {C}_{n}^{e}}{\partial {\lambda}}|_{\hat{\lambda}} \cdot {\delta}{\lambda} \\ \tag{3.2.15}
$$


名义预测位置为：
$$
\begin{align}
\hat{r}_G^e
&= \hat{r}_I^e + \hat{C}_{n}^e \cdot \hat{C}_b^n \cdot l^b \\
\end{align} \tag{3.2.16}
$$

位置观测：
$$
\tilde{r}_G^e = r_G^e + C_{n}^e \cdot n_r^n, \quad n_{r,k}^{n} \sim N(0, R_k) \tag{3.2.17}
$$

位置残差 = 观测 - 名义预测：
$$
\begin{align}

\delta z_r
&=
\hat{C}_{e}^n \cdot (\tilde{r}_G^e - \hat{r}_G^e) \\
&=
\hat{C}_{e}^n \cdot [ \tilde{r}_G^e - (\hat{r}_I^e + \hat{C}_{n}^e \cdot \hat{C}_b^n \cdot l^b) ] \quad(残差计算) \\
&=
\hat{C}_{e}^n \cdot
[
(r_G^e + C_{n}^e \cdot n_r^{n})
-
(\hat{r}_I^e + \hat{C}_{n}^e \cdot \hat{C}_b^n \cdot l^b)
] \\

&=
\hat{C}_{e}^n \cdot
[
(
\hat{r}_I^e + {\delta}r_I^e +
\hat{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ \hat{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
+ {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ {\delta}{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
+ C_{n}^e \cdot n_r^{n}
)
-
(
\hat{r}_I^e + \hat{C}_{n}^e \cdot \hat{C}_b^n \cdot l^b
)
] \\
\\
&=
\hat{C}_{e}^n \cdot
(
{\delta}r_I^e
+ \hat{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
+ {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b + C_{n}^e \cdot n_r^{n} \\
&+ {\delta}{C}_{n}^{e} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b \quad(忽略二阶小项) ) \\
\\
&=
\hat{C}_{e}^n \cdot
[
{\delta}r_I^e
-\hat{C}_{n}^{e} \cdot (\hat{C}_{b}^{n} \cdot l^b) ^{\wedge} \cdot {\phi}
+ {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ C_{n}^e \cdot n_r^{n}
] \\

&=
\hat{C}_{e}^n \cdot {\delta}r_I^e
- \hat{C}_{e}^n \cdot \hat{C}_{n}^{e} \cdot (\hat{C}_{b}^{n} \cdot l^b) ^{\wedge} \cdot {\phi}
+ \hat{C}_{e}^n \cdot {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ \hat{C}_{e}^n \cdot C_{n}^e \cdot n_r^{n} \\

&=
{\delta}r_I^n
- (\hat{C}_{b}^{n} \cdot l^b) ^{\wedge} \cdot {\phi}
+ \hat{C}_{e}^n \cdot {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ \hat{C}_{e}^n \cdot (\hat{C}_{n}^e + {\delta}{C}_{n}^e) \cdot n_r^{n}
\\

&=
{\delta}r_I^n
- (\hat{C}_{b}^{n} \cdot l^b) ^{\wedge} \cdot {\phi}
+ \hat{C}_{e}^n \cdot {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ \hat{C}_{e}^n \cdot \hat{C}_{n}^e \cdot n_r^{n} + \hat{C}_{e}^n \cdot {\delta}{C}_{n}^e \cdot n_r^{n}
\\

&=
{\delta}r_I^n
- (\hat{C}_{b}^{n} \cdot l^b) ^{\wedge} \cdot {\phi}
+ \hat{C}_{e}^n \cdot {\delta}{C}_{n}^{e} \cdot \hat{C}_{b}^{n} \cdot l^b
+ n_r^{n}
\\

\end{align} \tag{3.2.18}
$$

> [!NOTE]
> ${\delta}{C}_{n}^e \cdot n_r^{n}$属于“状态误差 * 观测噪声”的二阶小量，一阶线性化时忽略。
> 

实际应用中，==忽略位置误差对$C_n^e$的影响（即忽略${\delta}C_n^e$）==

则**位置观测矩阵**为：
$$
H_r =
\begin{bmatrix}
I_{3\times3} & 0_{3\times3} & - (\hat{C}_b^n \cdot l^b)^{\wedge} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.19}
$$

**观测噪声矩阵**为：
$$
R_{r}
=
\begin{bmatrix}
{\sigma}_{N}^{2} & & \\
& {\sigma}_{E}^{2} & \\
& & {\sigma}_{D}^{2} \\
\end{bmatrix} \tag{3.2.20}
$$
- 位置观测噪声可以直接从接收机输出获取


###### Velocity

在导航系（NED）下建立GNSS天线相位中心与IMU测量中心的位置关系，

理论：
$$
r_G^n = r_I^n + C_b^n \cdot l^b \tag{3.2.21}
$$
对时间求导数：
$$
\begin{align}

\dot{r}_G^n &= \dot{r}_I^n + \frac{d (C_b^n \cdot l^b)}{dt} \\
{v}_G^n &= {v}_I^n + \dot{C}_b^n \cdot l^b + {C}_b^n \cdot \dot{l}^b \\
&= {v}_I^n + {C}_b^n \cdot (\omega _{nb}^b)^{\wedge} \cdot l^b \\
&= {v}_I^n + {C}_b^n \cdot [ (\omega _{ib}^{b} - C_n^b \cdot \omega _{in}^{n}) \times l^b ] \\
&= {v}_I^n + {C}_b^n \cdot [ \omega _{ib}^{b} \times l^b - (C_n^b \cdot \omega _{in}^{n}) \times l^b ] \\
&= {v}_I^n + {C}_b^n \cdot (\omega _{ib}^{b} \times l^b) - {C}_b^n \cdot [ (C_n^b \cdot \omega _{in}^{n}) \times l^b ] \\
&= {v}_I^n + {C}_b^n \cdot (\omega _{ib}^{b} \times l^b) - ({C}_b^n \cdot C_n^b \cdot \omega _{in}^{n}) \times ({C}_b^n \cdot l^b) \\
&= {v}_I^n + {C}_b^n \cdot (\omega _{ib}^{b} \times l^b) - {\omega _{in}^{n}}^{\wedge} \cdot {C}_b^n \cdot l^b \\
\end{align} \tag{3.2.22}
$$


因此，导航系（NED）下GNSS天线相位中心与IMU测量中心的真实速度关系为：
$$
\begin{align}
v_G^n
&=
{v}_I^n + {C}_b^n \cdot ({\omega}_{ib}^{b} \times l^b) - ({\omega}_{in}^{n})^{\wedge} \cdot {C}_b^n \cdot l^b
\\

&=
(\hat{v}_I^n + {\delta}{v}_I^n)
+ (I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^n \cdot [(\hat{\omega}_{ib}^{b} + {\delta}{\omega}_{ib}^{b}) \times l^b]
- (\hat{\omega}_{in}^{n} + {\delta}{\omega}_{in}^{n})^{\wedge} \cdot (I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^n \cdot l^b
\\
\\
&=
\hat{v}_I^n + {\delta}{v}_I^n \\
&+ \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
+ \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b) \\
&- (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
\\
\\
&=
\hat{v}_I^n + {\delta}{v}_I^n
+ \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
+ \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
\\
&+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b \quad(忽略二阶小项)
\end{align} \tag{3.2.23}
$$

名义预测速度为：
$$
\begin{align}
\hat{v}_G^n &=
\hat{v}_I^n + \hat{C}_b^n \cdot (\hat{\omega}_{ib}^{b} \times l^b) - (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
\\
\end{align} \tag{3.2.24}
$$

速度观测：
$$
\tilde{v}_G^n = {v}_G^n + n_v^n,\quad n_{v,k} \sim N(0, R_k) \tag{3.2.25}
$$

速度残差 = 观测 - 名义预测：
$$
\begin{align}

\delta z_v
&=
\tilde{v}_G^n - \hat{v}_G^n \\
&=
\tilde{v}_G^n
- [
\hat{v}_I^n + \hat{C}_b^n \cdot (\hat{\omega}_{ib}^{b} \times l^b) - (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
] \quad(残差计算) \\

&=
[
{v}_G^n + n_v^n
]
-
[
\hat{v}_I^n + \hat{C}_b^n \cdot (\hat{\omega}_{ib}^{b} \times l^b) - (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
]
\\
\\
&=
[
\hat{v}_I^n + {\delta}{v}_I^n
+ \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
+ \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
+ n_v^n
]
\\
&-
[
\hat{v}_I^n + \hat{C}_b^n \cdot (\hat{\omega}_{ib}^{b} \times l^b) - (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
]
\\
\\
&=
{\delta}{v}_I^n
+ \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
- ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^n \cdot l^b
+ n_v^n
\\

&=
{\delta}{v}_I^n
+ \{ (\hat{\omega}_{in}^{n})^{\wedge} \cdot (\hat{C}_{b}^n \cdot l^b)^{\wedge}
- [ \hat{C}_{b}^n \cdot ( {\omega}_{ib}^{b} \times l^b ) ]^{\wedge} \} \cdot {\phi}
+ n_v^n
+ \hat{C}_{b}^n \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ (\hat{C}_{b}^n \cdot l^b)^{\wedge} \cdot {\delta}{\omega}_{in}^{n}
\\

&=
{\delta}{v}_I^n
+ \{ (\hat{\omega}_{in}^{n})^{\wedge} \cdot (\hat{C}_{b}^n \cdot l^b)^{\wedge}
- [ \hat{C}_{b}^n \cdot ( {\omega}_{ib}^{b} \times l^b ) ]^{\wedge} \} \cdot {\phi}
+ n_v^n
+ \hat{C}_{b}^n \cdot (-{\delta}b_{g}^{b} \times l^b)
+ (\hat{C}_{b}^n \cdot l^b)^{\wedge} \cdot {\delta}{\omega}_{in}^{n}
\\
&=
{\delta}{v}_I^n
+ \{ (\hat{\omega}_{in}^{n})^{\wedge} \cdot (\hat{C}_{b}^n \cdot l^b)^{\wedge}
- [ \hat{C}_{b}^n \cdot ( {\omega}_{ib}^{b} \times l^b ) ]^{\wedge} \} \cdot {\phi}
+ n_v^n
+ \hat{C}_{b}^n \cdot [(l^b)^{\wedge} \cdot {\delta}b_{g}^{b}]
+ (\hat{C}_{b}^n \cdot l^b)^{\wedge} \cdot {\delta}{\omega}_{in}^{n}
\\
&=
{\delta}{v}_I^n
+ \{ (\hat{\omega}_{in}^{n})^{\wedge} \cdot (\hat{C}_{b}^n \cdot l^b)^{\wedge}
- [ \hat{C}_{b}^n \cdot ( {\omega}_{ib}^{b} \times l^b ) ]^{\wedge} \} \cdot {\phi}
+ [\hat{C}_{b}^n \cdot (l^b)^{\wedge}] \cdot {\delta}b_{g}^{b}
+ n_v^n\\
&+ (\hat{C}_{b}^n \cdot l^b)^{\wedge} \cdot {\delta}{\omega}_{in}^{n} \quad(小量，实际应用时可忽略)
\\

\end{align} \tag{3.2.26}
$$


> [!NOTE]
> ${\delta}{\omega}_{ib}^{b} \approx -{\delta}b_g^b$：
> 忽略陀螺仪噪声：属于过程噪声，在预测步骤处理。

则**速度观测矩阵**为：
$$
\begin{align}
H_v =
\begin{bmatrix}
0_{3\times3} &
I_{3\times3} &
(\hat{\omega}_{in}^{n})^{\wedge} \cdot (\hat{C}_{b}^n \cdot l^b)^{\wedge}
- [ \hat{C}_{b}^n \cdot ( {\omega}_{ib}^{b} \times l^b ) ]^{\wedge} &
\hat{C}_{b}^n \cdot (l^b)^{\wedge} &
0_{3\times3}
\end{bmatrix}

\end{align} \tag{3.2.27}
$$

**观测噪声矩阵**为：
$$
R_{v}
=
\begin{bmatrix}
{\sigma}_{v,N}^{2} & & \\
& {\sigma}_{v,E}^{2} & \\
& & {\sigma}_{v,D}^{2} \\
\end{bmatrix} \tag{3.2.28}
$$
- 速度观测噪声可以直接从接收机输出获取

##### Tight-Couping

###### Position


###### Velocity



#### NH/ODO

前提假设：
- GNSS与NH/ODO中的$l^b$不同，此处不作符号区分；
- 对于NHC，仅考虑非前向速度；


导航系（NED）下NH/ODO测量中心与IMU测量中心的真实速度关系为：
$$
\begin{align}

{v}_O^n
&=
{v}_I^n +
{C}_b^n \cdot (\omega _{ib}^{b} \times l^b)
- (\omega _{in}^{n})^{\wedge} \cdot {C}_b^n \cdot l^b \\

\\

{v}_O^v
&=
{C}_b^v \cdot {C}_n^b \cdot {v}_I^n
+ {C}_b^v \cdot {C}_n^b \cdot {C}_b^n \cdot (\omega _{ib}^{b} \times l^b)
- {C}_b^v \cdot {C}_n^b \cdot (\omega _{in}^{n})^{\wedge} \cdot {C}_b^n \cdot l^b \\

&=
{C}_b^v \cdot {C}_n^b \cdot {v}_I^n
+ {C}_b^v \cdot ({\omega}_{ib}^{b} \times l^b)
- {C}_b^v \cdot {C}_n^b \cdot ({\omega}_{in}^{n})^{\wedge} \cdot {C}_b^n \cdot l^b \\

\\

&=
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (I - {\phi}^{\wedge}) \cdot (\hat{v}_I^n + {\delta}{v}_I^n) \\
&+ {C}_b^v \cdot [(\hat{\omega}_{ib}^{b} + {\delta}{\omega}_{ib}^{b}) \times l^b] \\
&- {C}_b^v \cdot \hat{C}_{n}^{b} \cdot (I - {\phi}^{\wedge}) \cdot (\hat{\omega}_{in}^{n} + {\delta}{\omega}_{in}^{n})^{\wedge} \cdot (I + {\phi}^{\wedge}) \cdot \hat{C}_{b}^{n} \cdot l^b \\

\end{align} \tag{3.2.29}
$$

- item 1
$$
\begin{align}

{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (I - {\phi}^{\wedge}) \cdot (\hat{v}_I^n + {\delta}{v}_I^n)
&=
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot \hat{v}_I^n
+ {C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\delta}{v}_I^n
- {C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot \hat{v}_I^n \\
&- {C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot {\delta}{v}_I^n \quad(忽略二阶小项)

\end{align} \tag{3.2.30}
$$

- item 2
$$
\begin{align}

{C}_b^v \cdot [(\hat{\omega}_{ib}^{b} + {\delta}{\omega}_{ib}^{b}) \times l^b]
&=
{C}_b^v \cdot ( \hat{\omega}_{ib}^{b} \times l^b + {\delta}{\omega}_{ib}^{b} \times l^b ) \\

&=
{C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b) + {C}_b^v \cdot ({\delta}{\omega}_{ib}^{b} \times l^b) \\

\end{align} \tag{3.2.31}
$$

- item 3
$$
\begin{align}

&=
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot
(I - {\phi}^{\wedge}) \cdot [ (\hat{\omega}_{in}^{n})^{\wedge} + ({\delta}{\omega}_{in}^{n})^{\wedge} ] \cdot (I + {\phi}^{\wedge})
\cdot \hat{C}_{b}^{n} \cdot l^b \\

&=
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot
[
(\hat{\omega}_{in}^{n})^{\wedge}
+ ({\delta}{\omega}_{in}^{n})^{\wedge}
- {\phi}^{\wedge} \cdot (\hat{\omega}_{in}^{n})^{\wedge}
- {\phi}^{\wedge} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge}

+(\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
+ ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
- {\phi}^{\wedge} \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
- {\phi}^{\wedge} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
]
\cdot \hat{C}_{b}^{n} \cdot l^b \\

&=
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot
[\\
&(\hat{\omega}_{in}^{n})^{\wedge}
+ ({\delta}{\omega}_{in}^{n})^{\wedge}
- {\phi}^{\wedge} \cdot (\hat{\omega}_{in}^{n})^{\wedge}
+(\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \\
&- {\phi}^{\wedge} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge}
+ ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
- {\phi}^{\wedge} \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge}
- {\phi}^{\wedge} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot {\phi}^{\wedge} \quad(忽略高阶小项) \\
&]
\cdot \hat{C}_{b}^{n} \cdot l^b \\

&=
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot
[
(\hat{\omega}_{in}^{n})^{\wedge} + ({\delta}{\omega}_{in}^{n})^{\wedge}
+ (\hat{\omega}_{in}^{n} \times {\phi})^{\wedge}
]
\cdot \hat{C}_{b}^{n} \cdot l^b \\

&=
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{\omega}_{in}^{n} \times {\phi})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b \\



\end{align} \tag{3.2.32}
$$



名义预测速度为：
$$
\begin{align}
\hat{v}_O^v
&=
{C}_b^v \cdot \hat{C}_n^b \cdot \hat{v}_I^n +
{C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- {C}_b^v \cdot \hat{C}_n^b \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
\\

\end{align} \tag{3.2.33}
$$

速度观测：
$$
\tilde{v}_O^v = {v}_O^v + n_O^v, \quad n_{O,k}^{v} \sim N(0, R_k) \tag{3.2.34}
$$

速度残差 = 观测 - 名义预测：
$$
\begin{align}

\delta z_v
&=
\tilde{v}_O^v - \hat{v}_O^v \\
&=
\tilde{v}_O^v -
[
{C}_b^v \cdot \hat{C}_n^b \cdot \hat{v}_I^n +
{C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- {C}_b^v \cdot \hat{C}_n^b \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
] \quad(残差计算) \\

&=
[
{v}_O^v + n_O^v
] \\
&-
[
{C}_b^v \cdot \hat{C}_n^b \cdot \hat{v}_I^n +
{C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- {C}_b^v \cdot \hat{C}_n^b \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
]
\\

\\

&=
[
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot \hat{v}_I^n
+ {C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\delta}{v}_I^n
- {C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot \hat{v}_I^n \\
&+ {C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b) + {C}_b^v \cdot ({\delta}{\omega}_{ib}^{b} \times l^b) \\
&-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
-{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{\omega}_{in}^{n} \times {\phi})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b
+ n_O^v
] \\
&-
[
{C}_b^v \cdot \hat{C}_n^b \cdot \hat{v}_I^n +
{C}_b^v \cdot (\hat{\omega}_{ib}^{b} \times l^b)
- {C}_b^v \cdot \hat{C}_n^b \cdot (\hat{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_b^n \cdot l^b
]
\\

\\

&=
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\delta}{v}_I^n
+ {C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{v}_I^n)^{\wedge} \cdot {\phi}
+ {C}_b^v \cdot ({\delta}{\omega}_{ib}^{b} \times l^b)
+ n_O^v \\
& -{C}_b^v \cdot \hat{C}_{n}^{b} \cdot ({\delta}{\omega}_{in}^{n})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b \quad(小量，实际应用时可忽略) \\
& -{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{\omega}_{in}^{n} \times {\phi})^{\wedge} \cdot \hat{C}_{b}^{n} \cdot l^b \quad(小量，实际应用时可忽略) \\

&=
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\delta}{v}_I^n
+ {C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{v}_I^n)^{\wedge} \cdot {\phi}
+ {C}_b^v \cdot [(-{\delta}b_{g}^{b}) \times l^b]
+ n_O^v \\

&=
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot {\delta}{v}_I^n
+ {C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{v}_I^n)^{\wedge} \cdot {\phi}
+ {C}_b^v \cdot [l^b \times {\delta}b_{g}^{b}]
+ n_O^v \\

&=
[{C}_b^v \cdot \hat{C}_{n}^{b}] \cdot {\delta}{v}_I^n
+ [{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{v}_I^n)^{\wedge}] \cdot {\phi}
+ [{C}_b^v \cdot (l^b)^{\wedge}] \cdot {\delta}b_{g}^{b}
+ n_O^v \\


\end{align} \tag{3.2.35}
$$

则**速度观测矩阵**为：
$$
\begin{align}
H_v =
\begin{bmatrix}
0_{3\times3} &
{C}_b^v \cdot \hat{C}_{n}^{b} &
{C}_b^v \cdot \hat{C}_{n}^{b} \cdot (\hat{v}_I^n)^{\wedge} &
{C}_b^v \cdot (l^b)^{\wedge} &
0_{3\times3}
\end{bmatrix}

\end{align} \tag{3.2.36}
$$

**观测噪声矩阵**为：
$$
R_{v,nh/odo}
=
\begin{bmatrix}
{\sigma}_{v,F}^{2} & & \\
& {\sigma}_{v,R}^{2} & \\
& & {\sigma}_{v,D}^{2} \\
\end{bmatrix} \tag{3.2.37}
$$

##### 自适应nh/odo观测噪声


#### Gyro-Bias

当环境温度变化时，陀螺的输出会偏离真实值，产生一个额外的零偏误差。对于MEMS陀螺仪，在工程上可以通过“**温度补偿**”技术来大幅降低其影响。

从温补表中获取的陀螺仪零偏作为观测值、获取的陀螺仪方差作为观测噪声：
$$
\widetilde{b}_{g}^{b,temp} = b_{g}^{b} + n_{g}^{b,temp}, \quad n_{g}^{b,temp} \sim \mathbf{N}[0, R_{b_g^b}] \\ \tag{3.2.38}
$$


真实陀螺仪零偏：
$$
b_g^b = \hat{b}_g^b + {\delta}{b}_g^b \tag{3.2.39}
$$

滤波器的实时估计作为名义预测值：$\hat{b}_g^b$


陀螺残差 = 观测 - 名义预测：
$$
\begin{align}

\delta z_{b_{g}}
&=
\widetilde{b}_{g}^{b,temp} - \hat{b}_{g}^{b} \quad(残差计算)
\\
&=
[b_{g}^{b} + n_{g}^{b,temp}] - \hat{b}_g^b \\

&=
\hat{b}_g^b + {\delta}{b}_g^b + n_{g}^{b,temp} - \hat{b}_g^b \\

&=
{\delta}{b}_g^b + n_{g}^{b,temp} \\

\end{align} \tag{3.2.40}
$$
陀螺仪零偏观测矩阵：
$$
H_{b_g^b} =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.41}
$$

观测噪声矩阵：
$$
R_{b_g}
=
\begin{bmatrix}
{\sigma}_{b_g^b,temp}^{2} & & \\
& {\sigma}_{b_g^b,temp}^{2} & \\
& & {\sigma}_{b_g^b,temp}^{2} \\
\end{bmatrix} \tag{3.2.42}
$$
- 观测噪声从**温补表的陀螺仪方差**获取



#### PVA-Fixed

- 理论上，静止状态下，位置和姿态保持不变、速度为0；
- 将静止时刻记录的位置/速度/姿态作为观测值、滤波器的实时估计作为名义预测值；


**位置：**

静止时真实位置保持不变，虚拟位置观测为$r_{fixed}^{e}$

位置观测值：
$$
\begin{align}
\widetilde{r}_{}^{e} &= {r}_{}^{e} + C_{n}^{e} \cdot {n}_{r,fixed}^{n} \\
{r}_{fixed}^{n} &= {r}_{}^{e} + C_{n}^{e} \cdot {n}_{r,fixed}^{n} \\
\end{align} \tag{3.2.43}
$$
真实位置与名义位置的关系：
$$
{r}_{}^{n} = \hat{r}_{}^{n} + {\delta}{r}_{}^{n} \\ \tag{3.2.44}
$$

位置残差 = 观测 - 名义预测：
$$
\begin{align}

{\delta}z_{r}
&=
\hat{C}_{e}^{n} \cdot (\widetilde{r}_{}^{e} - \hat{r}_{}^{e}) \\

&=
\hat{C}_{e}^{n} \cdot (\widetilde{r}_{}^{e} - ) \\

&=
{r}_{fixed}^{n} - \hat{r}_{}^{n} \quad(残差计算) \\

&=
{r}_{fixed}^{n} - ({r}_{}^{n} - {\delta}{r}_{}^{n}) \\

&=
({r}_{}^{n} + {n}_{r,fixed}^{n}) - ({r}_{}^{n} - {\delta}{r}_{}^{n}) \\

&=
{\delta}{r}_{}^{n} + {n}_{r,fixed}^{n} \\

\end{align} \tag{3.2.45}
$$
位置观测矩阵：
$$
H_{r,fixed} =
\begin{bmatrix}
I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.46}
$$

观测噪声矩阵：
$$
R_{r,fixed}
=
\begin{bmatrix}
{\sigma}_{N,fixed}^{2} & & \\
& {\sigma}_{E,fixed}^{2} & \\
& & {\sigma}_{D,fixed}^{2} \\
\end{bmatrix} \tag{3.2.47}
$$



**速度：**

静止时真实速度为${v}_{}^{n} = 0_{3\times3}$，虚拟速度观测为$v_{fixed}^{n} = 0_{3\times3}$

速度观测值：
$$
\begin{align}
\widetilde{v}_{}^{n} &= 0 \\
&= {v}_{}^{n} + {n}_{v,fixed}^{n} \\
\end{align} \tag{3.2.48}
$$

真实速度与名义速度的关系：
$$
\begin{align}
{v}_{}^{n} &= \hat{v}_{}^{n} + {\delta}{v}_{}^{n} \\
\end{align} \tag{3.2.49}
$$

速度残差 = 观测 - 名义预测：
$$
\begin{align}
{\delta}z_{v} &= \widetilde{v}_{}^{n} - \hat{v}_{}^{n} \\
&= 0 - \hat{v}_{}^{n} \quad(残差计算) \\
&= -({v}_{}^{n} - {\delta}{v}_{}^{n}) \\
&=-(-{n}_{v,fixed}^{n} - {\delta}{v}_{}^{n}) \\
&={\delta}{v}_{}^{n} + {n}_{v,fixed}^{n} \\
\end{align} \tag{3.2.50}
$$
位置观测矩阵：
$$
H_{v,fixed} =
\begin{bmatrix}
0_{3\times3} & I_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.51}
$$

观测噪声矩阵：
$$
\begin{align}
R_{v,fixed}
&=
\begin{bmatrix}
{\sigma}_{v,N,fixed}^{2} & & \\
& {\sigma}_{v,E,fixed}^{2} & \\
& & {\sigma}_{v,D,fixed}^{2} \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
{\sigma}_{v,hor,zupt}^{2} & & \\
& {\sigma}_{v,hor,zupt}^{2} & \\
& & {\sigma}_{v,ver,zupt}^{2} \\
\end{bmatrix}
\end{align} \tag{3.2.52}
$$


**姿态：**

静止时真实姿态保持不变，虚拟姿态观测为${C}_{b,fixed}^{n}$

真实姿态与名义姿态的关系：
$$
\begin{align}

{C}_{b}^{n}
&= {C}_{n'}^{n} \cdot \hat{C}_{b}^{n} \\
&= \mathbf{Exp}(\phi) \cdot \hat{C}_{b}^{n} \\

\end{align} \tag{3.2.53}
$$

姿态观测：
$$
\begin{align}

\widetilde{C}_{b}^{n}
&= {C}_{b,fixed}^{n} \\
&= {C}_{b}^{n} \cdot \mathbf{Exp}(n_{\theta}^{b}) \\
&= \mathbf{Exp}(\phi) \cdot \hat{C}_{b}^{n} \cdot \mathbf{Exp}(n_{\theta}^{b}) \\

\\

\widetilde{C}_{b}^{n} \cdot (\hat{C}_{b}^{n})^T
&=
\mathbf{Exp}(\phi) \cdot \hat{C}_{b}^{n} \cdot \mathbf{Exp}(n_{\theta}^{b}) \cdot (\hat{C}_{b}^{n})^T \\
&=
\mathbf{Exp}(\phi) \cdot \mathbf{Exp}( \hat{C}_{b}^{n} \cdot n_{\theta}^{b} ) \\
&\approx \mathbf{Exp}( {\phi} + \hat{C}_{b}^{n} \cdot n_{\theta}^{b} ) \\

\\

\mathbf{Log}[ \widetilde{C}_{b}^{n} \cdot (\hat{C}_{b}^{n})^T ]
&\approx
{\phi} + \hat{C}_{b}^{n} \cdot n_{\theta}^{b} \\

\end{align} \tag{3.2.54}
$$

姿态误差为：
$$
\begin{align}
z_{\phi}
&= \mathbf{Log}[ \widetilde{C}_{b}^{n} \cdot (\hat{C}_{b}^{n})^T ] \\
&= \mathbf{Log}[ {C}_{b,fixed}^{n} \cdot (\hat{C}_{b}^{n})^T ] \quad(残差计算) \\
&\approx {\phi} + \hat{C}_{b}^{n} \cdot n_{\theta}^{b} \\

\end{align} \tag{3.2.55}
$$
姿态观测矩阵为：
$$
H_{\phi,fixed} =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & I_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.56}
$$
观测噪声矩阵：
$$
\begin{align}
R_{\phi,fixed}
&=
\hat{C}_{b}^{n} \cdot R_{\theta} \cdot (\hat{C}_{b}^{n})^T \\
&=
\hat{C}_{b}^{n} \cdot
\begin{bmatrix}
{\sigma}_{\theta,fixed}^{2} & & \\
& {\sigma}_{\theta,fixed}^{2} & \\
& & {\sigma}_{\theta,fixed}^{2} \\
\end{bmatrix}
\cdot (\hat{C}_{b}^{n})^T \\

\end{align} \tag{3.2.57}
$$


#### ZARU(Zero Angular Rate Update)

已知 
$$
\omega _{nb}^{b} = \omega _{ib}^{b} - C_n^b \cdot (\omega _{ie}^{n} + \omega _{en}^{n}) \tag{3.2.58}
$$
在静止状态下，有： 
$$
\begin{align}
\omega _{nb}^{b} &= 0 \\
\omega _{en}^{n} &= 0 \\
\end{align} \tag{3.2.59}
$$
则真实角速率为： 
$$
\begin{align}

\omega _{ib}^{b}
&= C_n^b \cdot {\omega}_{ie}^{n} \\
&= \hat{C}_{n}^{b} \cdot (I - {\phi}^{\wedge}) \cdot ( \hat{\omega}_{ie}^{n} + {\delta}{\omega}_{ie}^{n} ) \\
&=
\hat{C}_{n}^{b} \cdot \hat{\omega}_{ie}^{n}
+ \hat{C}_{n}^{b} \cdot {\delta}{\omega}_{ie}^{n}
- \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot \hat{\omega}_{ie}^{n}
- \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot {\delta}{\omega}_{ie}^{n} \\

&=
\hat{C}_{n}^{b} \cdot \hat{\omega}_{ie}^{n}
+ [ \hat{C}_{n}^{b} \cdot (\hat{\omega}_{ie}^{n})^{\wedge} ] \cdot {\phi} \\
&+ \hat{C}_{n}^{b} \cdot {\delta}{\omega}_{ie}^{n} \quad(忽略小量) \\
&- \hat{C}_{n}^{b} \cdot {\phi}^{\wedge} \cdot {\delta}{\omega}_{ie}^{n} \quad(忽略二阶小项) \\
\end{align} \tag{3.2.60}
$$

名义角速率预测：
$$
\hat{\omega}_{ib}^{b} = \hat{C}_n^b \cdot \hat{\omega}_{ie}^{n} \\ \tag{3.2.61}
$$

角速率观测（工程上，进行观测更新时，==已补偿零偏==）：
$$
\begin{align}
\widetilde{\omega}_{ib}^{b}
&= {\omega}_{ib}^b + b_g^b + n_{g}^b - \hat{b}_g^b \\
&= {\omega}_{ib}^b + {\delta}{b}_g^b + n_{g}^b \\
\end{align} \tag{3.2.62}
$$

角速率残差 = 观测 - 名义预测：
$$
\begin{align}

z_{{\omega}_{ib}^{b}}
&=
\widetilde{\omega}_{ib}^{b} - \hat{\omega}_{ib}^{b} \\
&=
\widetilde{\omega}_{ib}^{b} - (\hat{C}_n^b \cdot \hat{\omega}_{ie}^{n}) \quad(残差计算) \\
&=
({\omega}_{ib}^b + {\delta}{b}_g^b + n_{g}^b)
-
(\hat{C}_n^b \cdot \hat{\omega}_{ie}^{n}) \\

&=
(
\hat{C}_{n}^{b} \cdot \hat{\omega}_{ie}^{n} + [ \hat{C}_{n}^{b} \cdot (\hat{\omega}_{ie}^{n})^{\wedge} ] \cdot {\phi}
+ {\delta}b_g^b + n_{g}^b
)
-
\hat{C}_n^b \cdot \hat{\omega}_{ie}^{n} \\

&=
[ \hat{C}_{n}^{b} \cdot (\hat{\omega}_{ie}^{n})^{\wedge} ] \cdot {\phi}
+ {\delta}{b}_{g}^b + n_{g}^b \\

\end{align} \tag{3.2.63}
$$

角速率观测矩阵：
$$
H_{{\omega}_{ib}^{b},fixed} =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & \hat{C}_{n}^{b} \cdot (\hat{\omega}_{ie}^{n})^{\wedge} & I_{3\times3} & 0_{3\times3}
\end{bmatrix} \tag{3.2.64}
$$
观测噪声矩阵：
$$
R_{{\omega}_{ib}^{b},fixed} =
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & {\sigma}_{g}^{2} \cdot I_{3\times3} & 0_{3\times3} \\
\end{bmatrix} \tag{3.2.65}
$$


### 3.2.2 序贯更新

- 每次仅对**一维数据**进行更新



* 1. 对**协方差矩阵P**进行`cholesky`分解： 
$$
P = L \cdot L^T \tag{3.2.66}
$$
- 2. **卡尔曼增益K**表示为：
$$
\begin{align}

K
&= P \cdot H^T \cdot (H \cdot P \cdot H^T + R)^{-1} \\
&= P \cdot H^T \cdot (H \cdot L \cdot L^T \cdot H^T + R)^{-1} \\
&= P \cdot H^T \cdot [(H \cdot L) \cdot (H \cdot L)^T + R]^{-1} \\
&= P \cdot H^T \cdot (a \cdot a^T + R)^{-1} \\
&= P \cdot H^T \cdot b^{-1} \\
&= L \cdot L^T \cdot H^T \cdot b^{-1} \\
&= L \cdot (H \cdot L)^{T} \cdot b^{-1} \\
&= L \cdot (a)^{T} \cdot b^{-1} \\
&= G \cdot b^{-1} \\

\end{align} \tag{3.2.67}
$$
其中，令：
- $a = H \cdot L$
- $b = a \cdot a^T+R$：新息的理论方差（标量）
- $G = L \cdot a^T$：未归一化增益


- **3. 状态更新公式为：**
$$
\begin{align}
X'
&= X + K \cdot (Z - H \cdot X) = X + K \cdot r \\
&= X + G \cdot b^{-1} \cdot r \\
&= X + G \cdot (\frac{r}{b}) \\
&= X + G \cdot dx \\
\end{align} \tag{3.2.68}
$$
其中，令：
- $r = Z - H \cdot X$：新息
- $dx = \frac{r}{b}$：归一化残差


- **4. 残差归一化，调整更新步长**

避免$b \rightarrow 0$导致的数值不稳定，引入步长因子：
$$
s = 1 + \sqrt{\frac{R}{b}} \\ \tag{3.2.69}
$$
构造等效归一化增益系数：
$$
\begin{align}
d &= s \cdot b \\
&= (1 + \sqrt{\frac{R}{b}}) \cdot b \\
&= b + \sqrt{R \cdot b} \\
\end{align} \tag{3.2.70}
$$

- **5. 秩-1 更新（`Sherman-Morrison & Cholesky分解`）**
$$
\begin{align}
L^{'} &= L - \frac{L \cdot a^T  \cdot a}{b + \sqrt{R \cdot b}} \\
&= L - \frac{G \cdot a}{d} \\
\end{align} \tag{3.2.71}
$$
- **6. 更新矩阵P**
$$
P = L^{'} \cdot {L^{'}}^T \tag{3.2.72}
$$

#### 详细推导

标准协方差更新可以写成：
$$
\begin{align}
P
&=
P - K \cdot (H \cdot P \cdot H^T + R) \cdot K^T \\

&=
P - K \cdot b \cdot K^T \\

&=
P - (G \cdot b^{-1}) \cdot b \cdot (G \cdot b^{-1})^T \\

&=
P - \frac{G \cdot G^T}{b} \\

&=
L \cdot L^T - \frac{(L \cdot a^T) \cdot (L \cdot a^T)^T}{b} \\

&=
L \cdot L^T - \frac{L \cdot a^T \cdot a \cdot L^T}{b} \\

&= L \cdot (I - \frac{a^T \cdot a}{b}) \cdot L^T \\

\end{align} \tag{3.2.73}
$$
令$\nu = \frac{a^T}{\sqrt{b}}$，则有：
$$
\begin{align}
P &= L \cdot (I - \nu \cdot {\nu}^T) \cdot L^T \\
\\

\Vert \nu \Vert^{2} &= {\nu}^T \cdot {\nu} = \frac{a \cdot a^T}{b} \\
&= \frac{b - R}{b} \\
&= 1 - \frac{R}{b} \\
\end{align} \tag{3.2.74}
$$
移项得到：
$$
\sqrt{1 - \Vert \nu \Vert^{2}} = \sqrt{\frac{R}{b}} \\ \tag{3.2.75}
$$
对于$\ref{eq:3.2.73}$形式的秩-1更新，目标是找到$L{'}$，使得：
$$
\begin{align}
P{'} = L{'} \cdot L{'}^T &= L \cdot (I - \nu \cdot {\nu}^T) \cdot L^T \\
\end{align} \tag{3.2.76}
$$
标准平方根下更新公式为：
$$
\begin{align}
L{'} &= L \cdot (I - \frac{1}{1 + \sqrt{1 - \Vert \nu \Vert^{2}}} \cdot \nu \cdot {\nu}^T)
\end{align} \tag{3.2.77}
$$
当$b / \frac{R}{b}$很小时，直接计算导致：
- $\frac{G \cdot G^T}{b}$很大，$P$减去一个很大的数，容易失去正定性；
- $1 - \Vert \nu \Vert^{2}$是2个接近的数相减，浮点计算严重损失精度；


因此定义：
$$
\begin{align}
s &= 1 + \sqrt{1 - \Vert \nu \Vert^{2}} \\
&= 1 + \sqrt{\frac{R}{b}} \\
\end{align} \tag{3.2.78}
$$
其中，$s$本质上是**秩-1更新中的稳定化分母因子**，把下更新分母写成数值稳定的形式，避免更新$L$时直接计算$1 - \Vert \nu \Vert^{2}$。


则$\ref{eq:3.2.77}$更新为：
$$
\begin{align}
L{'} &= L \cdot (I - \frac{1}{s} \cdot \frac{a^T \cdot a}{b}) \\
&= L \cdot (I - \frac{a^T \cdot a}{s \cdot b}) \\
&= L - \frac{L \cdot a^T \cdot a}{s \cdot b} \\
\end{align} \tag{3.2.79}
$$
定义：
$$
\begin{align}
d &= s \cdot b \\
&= (1 + \sqrt{\frac{R}{b}}) \cdot b \\
&= b + \sqrt{R \cdot b} \\
\end{align} \tag{3.2.80}
$$
- $d$是**稳定化后的等效分母**，避免除以小量；
- 更新系数$\frac{1}{s}$有界，不会出现增益爆炸；


## 3.3 后验误差状态反馈

### 位置

- $\delta h$向上为正、导航系为NED时${\delta}r_{D}^{n}$向下为正

将${\delta}r_{}^{n}$（ECEF位置误差投影到导航系）转换为llh增量：
$$
\begin{align}

{\delta}{L} &= \frac{{\delta}r_{N}^{n}}{R_{M} + h} \\
{\delta}{\lambda} &= \frac{{\delta}r_{E}^{n} \cdot \sec{L}}{R_{N} + h} \\
{\delta}{h} &= -{\delta}r_{D}^{n} \\
\end{align} \tag{3.3.1}
$$

- **方法一：直接对LLH进行反馈校正**
$$
\begin{align}

\hat{L}^{+} &= \hat{L}^{-} + {\delta}{L} \\
\hat{\lambda}^{+} &= \mathbf{Warp}(\hat{\lambda}^{-} + {\delta}{\lambda}) \\
\hat{h}^{+} &= \hat{h}^{-} + {\delta}{h} \\

\end{align} \tag{3.3.2}
$$


- **方法二：先更新$q_n^e$再反解LLH**

当载体从$(L, \lambda)$移动到$(L + {\delta}L, \lambda + {\delta}\lambda)$时，可以理解为地球**绕旋转轴旋转一个小角度(旋转向量)**，将载体从旧位置带到新位置。在地球球面上**从旧位置主动旋转到新位置**的等效旋转向量（方向：旋转轴，模长：旋转角）在旧位置的表示为：
$$
\mathbf{d}{\theta}_{n' \to n}^{n'}
=
\begin{bmatrix}
{\delta}{\lambda} \cdot \cos{\hat{L}^{-}} \\
- {\delta}{L} \\
- {\delta}{\lambda} \cdot \sin{\hat{L}^{-}} \\
\end{bmatrix} \tag{3.3.3}
$$

主动旋转矩阵：$R_{n'}^{n} = \mathbf{Exp}(\mathbf{d}{\theta}_{n' \to n}^{n'})$
坐标变换矩阵：$C_{n'}^{n} = \mathbf{Exp}(-\mathbf{d}{\theta}_{n' \to n}^{n'})$


将旋转向量用**旋转角 + 单位旋转轴**表示：
$$
\begin{align}
\mathbf{d}{\theta}_{n' \to n}^{n'} &= {\theta} \cdot \mu \\
{\theta} &= \Vert \mathbf{d}{\theta}_{n' \to n}^{n'} \Vert \\
\mu &= \frac{\mathbf{d}{\theta}_{n' \to n}^{n'}}{\Vert \mathbf{d}{\theta}_{n' \to n}^{n'} \Vert} \\
\end{align} \tag{3.3.4}
$$
通过指数映射将旋转向量转换为四元数：
$$
\begin{align}
q_{n^{+}}^{n^{-}}
&= (q_{n^{-}}^{n^{+}})^{-1} \\
&= (\mathbf{Exp}(-\mathbf{d}{\theta}_{n' \to n}^{n'}))^{-1} \\
&=\mathbf{Exp}(\mathbf{d}{\theta}_{n' \to n}^{n'}) \\
&=
\begin{bmatrix}
\cos(\frac{\theta}{2}) \\
\mu \cdot \sin(\frac{\theta}{2}) \\
\end{bmatrix}
\\
&\approx
\begin{bmatrix}
1 \\
\frac{1}{2} \cdot \mathbf{d}{\theta}^n \\
\end{bmatrix}
\quad(\theta为小量时近似)

\end{align} \tag{3.3.5}
$$

将载体从旧位置更新到新位置：
$$
\begin{align}
q_{n^{+}}^{e} &= q_{n^{-}}^{e} \otimes q_{n^{+}}^{n^{-}} \\
\end{align} \tag{3.3.6}
$$

归一化，避免数值漂移：
$$
q_{n^{+}}^{e} = \mathbf{normalize}(q_{n^{+}}^{e}) \tag{3.3.7}
$$


将$q_{n^{+}}^{e}$转换为$(\hat{L}^{+}, \hat{\lambda}^{+}, \hat{h}^{+})$：
$$
\begin{align}
\hat{L}^{+} &= -2 \cdot \mathbf{atan2}(q_{n^{+},y}^{e}, q_{n^{+},w}^{e}) - \frac{\pi}{2} \\
\hat{\lambda}^{+} &= \mathbf{Warp}[2 \cdot \mathbf{atan2}(q_{n^{+},z}^{e}, q_{n^{+},w}^{e})] \\
\hat{h}^{+} &= \hat{h}^{-} - {\delta}r_{D}^{n} \\
\end{align} \tag{3.3.8}
$$

> [!qne2ll]
> 假设导航系为NED，则有：
> $$
> \begin{align}
> q_n^e &= 
> \begin{bmatrix}
> w & x & y & z
> \end{bmatrix}^T
> \\
> \\
> C_n^e &=
> \begin{bmatrix}
> - \sin{L} \cdot \cos{\lambda} & -\sin{\lambda} & -\cos{L} \cdot \cos{\lambda} \\
> - \sin{L} \cdot \sin{\lambda} & \cos{\lambda} & -\cos{L} \cdot \sin{\lambda} \\
> \cos{L} & 0 & -\sin{L} \\
> \end{bmatrix}
> \end{align}
> $$
> 
> 旋转矩阵与四元数的标准转换关系为：
> $$
> R(q) = 
> \begin{bmatrix}
> 1 - 2 \cdot (y^2 + z^2) & 2 \cdot (x \cdot y - w \cdot z) & 2 \cdot (x \cdot z + w \cdot y) \\
> 2 \cdot (x \cdot y + w \cdot z) & 1 - 2 \cdot (x^2 + z^2) & 2 \cdot (y \cdot z - w \cdot x) \\
> 2 \cdot (x \cdot z - w \cdot y) & 2 \cdot (y \cdot z + w \cdot x) & 1 - 2 \cdot (x^2 + y^2) \\
> \end{bmatrix}
> $$
> 
> 对照$C_n^e$，有：
> $$
> \begin{align}
> R_{20} &= 2 \cdot (x \cdot z - w \cdot y) = \cos{L} \\
> R_{22} &= 1 - 2 \cdot (x^2 + y^2) = -\sin{L} \\
> R_{01} &= 2 \cdot (x \cdot y - w \cdot z) = -\sin{\lambda} \\
> R_{11} &= 1 - 2 \cdot (x^2 + z^2) = \cos{\lambda} \\
> \end{align}
> $$
> 
> 得：
> $$
> \begin{align}
> L 
> &= \mathbf{atan2}(-R_{22}, R_{20}) \\
> 
> 
> \lambda 
> &= \mathbf{Warp}[\mathbf{atan2}(-R_{01}, R_{11})] \\
> \end{align}
> $$
> 
> 利用四元数分量，可以简化为：
> $$
> \begin{align}
> L &= -2 \cdot \mathbf{atan2}(y, w) - \frac{\pi}{2} \\
> \lambda &= \mathbf{Warp}[2 \cdot \mathbf{atan2}(z, w)] \\
> \end{align}
> $$


|             |          way 1           |    way 2     |
| :---------: | :----------------------: | :----------: |
|    经度跳变     |    需要$\mathbf{Warp}$     |     自动处理     |
|    大误差精度    |         一阶近似，误差大         |     球面精确     |
|    极点附近     | $\cos{\lambda} \to 0$时发散 |  存在数值问题，更稳健  |
|    长时间运行    |           误差累积           |  保持旋转流形一致性   |
| 与$C_e^n$一致性 |           重新计算           | 直接由$q_n^e$给出 |


#### dTheta的完整推导

$C_e^n$是关于$[L, \lambda]$的函数，导航系为NED时的形式为：
$$
\begin{align}

C_e^n
&=
\begin{bmatrix}
-\sin{L} \cdot \cos{\lambda} & -\sin{L} \cdot \sin{\lambda} & \cos{L} \\
-\sin{\lambda} & \cos{\lambda} & 0 \\
-\cos{L} \cdot \cos{\lambda} & -\cos{L} \cdot \sin{\lambda} & -\sin{L} \\
\end{bmatrix}
\\

\\

C_n^e
&=
\begin{bmatrix}
e_N & e_E & e_D
\end{bmatrix}
\\
&=
\begin{bmatrix}
-\sin{L} \cdot \cos{\lambda} & -\sin{\lambda} & -\cos{L} \cdot \cos{\lambda} \\
-\sin{L} \cdot \sin{\lambda} & \cos{\lambda} & -\cos{L} \cdot \sin{\lambda} \\
\cos{L} & 0 & -\sin{L} \\
\end{bmatrix}

\end{align} \tag{3.3.9}
$$

在球形地球模型下，$C_e^n$不依赖$h$，因此${\delta}h$不会进入等效旋转矢量。若为椭球地球模型，$h$通过曲率半径间接影响，但一阶展开时$\frac{\partial C_n^e}{\partial h} = 0$成立。

对于新位置$[L+{\delta}L, {\lambda}+{\delta}{\lambda}, h+{\delta}h]$，一阶泰勒展开：
$$
\begin{align}

C_{n}^{e}(L+{\delta}L, {\lambda}+{\delta}{\lambda}, h+{\delta}h)
&\approx
C_{n}^{e}(L, {\lambda}, h) +
\frac{\partial C_{n}^{e}}{\partial L} \cdot {\delta}L +
\frac{\partial C_{n}^{e}}{\partial {\lambda}} \cdot {\delta}{\lambda} +
\frac{\partial C_{n}^{e}}{\partial h} \cdot {\delta}h \\

&\approx
C_{n}^{e}(L, {\lambda}, h) +
\frac{\partial C_{n}^{e}}{\partial L} \cdot {\delta}L +
\frac{\partial C_{n}^{e}}{\partial {\lambda}} \cdot {\delta}{\lambda} \\

&\approx
C_{n}^{e}(L, {\lambda}, h) + {\delta}C_{n}^{e} \\
\\

\frac{\partial C_{n}^{e}}{\partial L}
&=
\begin{bmatrix}
\frac{\partial e_N}{\partial L} & \frac{\partial e_E}{\partial L} & \frac{\partial e_D}{\partial L} \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
-\cos{L} \cdot \cos{\lambda} & 0 & \sin{L} \cdot \cos{\lambda} \\
-\cos{L} \cdot \sin{\lambda} & 0 & \sin{L} \cdot \sin{\lambda} \\
-\sin{L} & 0 & -\cos{L} \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
e_D & 0 & -e_N
\end{bmatrix}
\\
\\

\frac{\partial C_{n}^{e}}{\partial {\lambda}}
&=
\begin{bmatrix}
\frac{\partial e_N}{\partial {\lambda}} & \frac{\partial e_E}{\partial {\lambda}} & \frac{\partial e_D}{\partial {\lambda}} \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
-\sin{L} \cdot e_E & \sin{L} \cdot e_N + \cos{L} \cdot e_D & -\cos{L} \cdot e_E \\
\end{bmatrix}
\\

\end{align} \tag{3.3.10}
$$

因此有：
$$
\begin{align}

{\delta}C_{n}^{e}
&=
\frac{\partial C_{n}^{e}}{\partial L} \cdot {\delta}L +
\frac{\partial C_{n}^{e}}{\partial {\lambda}} \cdot {\delta}{\lambda}
\\
&=
\begin{bmatrix}
e_D \cdot {\delta}L -\sin{L} \cdot e_E \cdot {\delta}{\lambda} &
\sin{L} \cdot e_N \cdot {\delta}{\lambda} + \cos{L} \cdot e_D \cdot {\delta}{\lambda} &
-e_N \cdot {\delta}L -\cos{L} \cdot e_E \cdot {\delta}{\lambda} \\
\end{bmatrix}

\end{align} \tag{3.3.11}
$$

$C_n^e$为正交矩阵，小变化量可以近似为：
$$
C_{n}^{e}(L+{\delta}L, {\lambda}+{\delta}{\lambda}, h+{\delta}h)
\approx
C_{n}^{e}(L, {\lambda}, h)
\cdot
\mathbf{Exp}( \mathbf{d}{\theta} ) \tag{3.3.12}
$$

联合$\ref{eq:3.3.10}$与$\ref{eq:3.3.12}$：
$$
\begin{align}

C_{n}^{e}(L, {\lambda}, h) + {\delta}C_{n}^{e}
&=
C_{n}^{e}(L, {\lambda}, h) \cdot \mathbf{Exp}( \mathbf{d}{\theta} )
\\
C_{e}^{n}(L, {\lambda}, h) \cdot C_{n}^{e}(L, {\lambda}, h) + C_{e}^{n}(L, {\lambda}, h) \cdot {\delta}C_{n}^{e}
&= C_{e}^{n}(L, {\lambda}, h) \cdot C_{n}^{e}(L, {\lambda}, h) \cdot \mathbf{Exp}( \mathbf{d}{\theta} )
\\
I + C_{e}^{n}(L, {\lambda}, h) \cdot {\delta}C_{n}^{e}
&\approx
I + (\mathbf{d}{\theta})^{\wedge}
\\
\\
(\mathbf{d}{\theta})^{\wedge}
&\approx
C_{e}^{n}(L, {\lambda}, h) \cdot {\delta}C_{n}^{e}
\\
&\approx
\begin{bmatrix}
e_N^T \\ e_E^T \\ e_D^T \\
\end{bmatrix}
\cdot
\begin{bmatrix}
e_D \cdot {\delta}L -\sin{L} \cdot e_E \cdot {\delta}{\lambda} &
\sin{L} \cdot e_N \cdot {\delta}{\lambda} + \cos{L} \cdot e_D \cdot {\delta}{\lambda} &
-e_N \cdot {\delta}L -\cos{L} \cdot e_E \cdot {\delta}{\lambda} \\
\end{bmatrix}
\\
\begin{bmatrix}
0 & -\mathbf{d}{\theta}_{D} & \mathbf{d}{\theta}_{E} \\
\mathbf{d}{\theta}_{D} & 0 & -\mathbf{d}{\theta}_{N} \\
-\mathbf{d}{\theta}_{E} & \mathbf{d}{\theta}_{N} & 0 \\
\end{bmatrix}
&\approx
\begin{bmatrix}
0 & \sin{L} \cdot {\delta}{\lambda} & -{\delta}{L} \\
-\sin{L} \cdot {\delta}{\lambda} & 0 & -\cos{L} \cdot {\delta}{\lambda} \\
{\delta}{L} & \cos{L} \cdot {\delta}{\lambda} & 0 \\
\end{bmatrix}


\end{align} \tag{3.3.13}
$$

得：
$$
\begin{align}

\mathbf{d}{\theta}_{n' \to n}^{n'}
&=
\begin{bmatrix}
\mathbf{d}{\theta}_{N} \\ \mathbf{d}{\theta}_{E} \\ \mathbf{d}{\theta}_{D} \\
\end{bmatrix}
\\
&=
\begin{bmatrix}
{\delta}{\lambda} \cdot \cos{L} \\ - {\delta}{L} \\ -{\delta}{\lambda} \cdot \sin{L} \\
\end{bmatrix}

\end{align} \tag{3.3.14}
$$



### 速度

在位置反馈校正完成后，导航系发生变化：
$$
C_{n,old}^{n,new} = \mathbf{Exp}(-\mathbf{d}{\theta}_{n,old}^{n,new}) \approx I - (\mathbf{d}{\theta}_{n,old}^{n,new})^{\wedge} \tag{3.3.15}
$$
则速度反馈校正为：
$$
\begin{align}
{v}_{+}^{n}
&= C_{n,old}^{n,new} \cdot ( {v}_{-}^{n} + {\delta}{v}_{}^{n} ) \\
&\approx [I - (\mathbf{d}{\theta}_{n,old}^{n,new})^{\wedge}] \cdot ( {v}_{-}^{n} + {\delta}{v}_{}^{n} ) \\
&\approx  {v}_{-}^{n} + {\delta}{v}_{}^{n}  \\
\end{align} \tag{3.3.16}
$$
- 导航系变化项为一阶小量，通常远小于速度误差
- 使用场景：位置修正量大（例如组合导航模式切换）、高速载体、高精度应用

### 姿态

由$\ref{eq:2.1.13}$和$\ref{eq:2.1.14}$得：
$$
\begin{align}
C_{b}^{n} &= C_{n'}^{n} \cdot \hat{C}_{b}^{n} = \mathbf{Exp}(+{\phi}) \cdot \hat{C}_{b}^{n} \\
q_{b}^{n} &= q_{n'}^{n} \cdot \hat{q}_{b}^{n} = \mathbf{Exp}(+{\phi}) \cdot \hat{q}_{b}^{n} \\
\end{align} \tag{3.3.17}
$$
因此反馈校正为：
$$
\begin{align}
\hat{C}_{b,+}^{n} &= \mathbf{Exp}(+{\phi}) \cdot \hat{C}_{b,-}^{n} \\
\hat{q}_{b,+}^{n} &= \mathbf{Exp}(+{\phi}) \otimes \hat{q}_{b,-}^{n} \\
\end{align} \tag{3.3.18}
$$

归一化，避免数值漂移：
$$
\hat{q}_{b,+}^{n} = \mathbf{normalize}(\hat{q}_{b,+}^{n}) \\ \tag{3.3.19}
$$



### 零偏

$$
\begin{align}

\hat{b}_{g,+}^{b} &= \hat{b}_{g,-}^{b} + {\delta}b_{g}^{b} \\
\hat{b}_{a,+}^{b} &= \hat{b}_{a,-}^{b} + {\delta}b_{a}^{b} \\

\end{align} \tag{3.3.20}
$$



