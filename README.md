# Automatic-control

## 2023年春夏学期自动控制原理课程讨论题  
### 课程老师-吴俊  
<br>

### 1.基于sinulink的二自由度机械臂运动仿真  
仿真思路：  

Matlab中有许多可以用来求解非线性方程组的方法，其中比较常用的用于求解非线性方程组的一个求解器是ode45。  

用simulink仿真最好需要的两个条件：  

1. 闭环系统中最好只有积分环节，没有微分环节，因为微分是非因果的。  
2. 仿真系统需为反馈系统。  

根据上述两个条件，我们需要对给出的公式做一定的操作，将$ \ddot{\theta}_{1} $和 $ \ddot{\theta}_{2} $的表达式求出，表示成有关其他变量的形式。  

根据已知的公式：  
$$
\begin{equation}
\begin{split}
& \tau_1 = \mathrm{\ddot{\theta}}_{1}\,l_{1}\,\left(m_{1}+m_{2}\right)+{l_{2}}^2\,m_{2}\,\left(\mathrm{\ddot{\theta}}_{1}+\mathrm{\ddot{\theta}}_{2}\right)+g\,l_{1}\,\cos\left(\theta _{1}\right)\,\left(m_{1}+m_{2}\right)+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\\&+l_{1}\,l_{2}\,m_{2}\,\cos\left(\theta _{2}\right)\,\left(2\,\mathrm{\ddot{\theta}}_{1}+\mathrm{\ddot{\theta}}_{2}\right)-{\mathrm{\dot{\theta}}_{2}}^2\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)-2\,\mathrm{\dot{\theta}}_{1}\,\mathrm{\dot{\theta}}_{2}\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)

\\
\\
& \tau_2 = l_{1}\,m_{2}\,\cos\left(\theta _{2}\right)\,{\mathrm{\ddot{\theta}}_{1}}^2\,l_{2}+l_{1}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2\,l_{2}+m_{2}\,\left(\theta _{1}+\theta _{2}\right)\,{l_{2}}^2+g\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\,l_{2}


\end{split}
\end{equation}
$$
我们发现公式中关于 $\ddot{\theta}_{1} $和 $ \ddot{\theta}_{2} $都是线性的，因此我们可以将上述公式写成如下矩阵运算的形式  
$$
\begin{equation}
\left[\begin{array}{c} \tau _{1}\\ \tau _{2} \end{array}\right] = 
\left[\begin{array}{cc} \left(m_{1}+m_{2}\right)\,{l_{1}}^2+2\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{1}\,l_{2}+m_{2}\,{l_{2}}^2 & m_{2}\,{l_{2}}^2+l_{1}\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{2}\\ m_{2}\,{l_{2}}^2+l_{1}\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{2} & {l_{2}}^2\,m_{2} 
\end{array}\right]
\left[\begin{array}{c} \ddot{\theta} _{1}\\ \ddot{\theta} _{2} \end{array}\right]
+\\
\left[\begin{array}{c} -l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2-2\,\mathrm{\dot{\theta}}_{1}\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+g\,l_{1}\,\cos\left(\theta _{1}\right)\,\left(m_{1}+m_{2}\right)+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\\ l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right) \end{array}\right]

\end{equation}
$$
令上述形式为  
$$
\begin{equation}
C = A
\left[\begin{array}{c} \ddot{\theta} _{1}\\ \ddot{\theta} _{2} \end{array}\right]
+B
\end{equation}
$$
其中各个矩阵为：  
$$
\begin{equation}
A = 
\left[\begin{array}{cc} \left(m_{1}+m_{2}\right)\,{l_{1}}^2+2\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{1}\,l_{2}+m_{2}\,{l_{2}}^2 & m_{2}\,{l_{2}}^2+l_{1}\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{2}\\ m_{2}\,{l_{2}}^2+l_{1}\,m_{2}\,\cos\left(\theta _{2}\right)\,l_{2} & {l_{2}}^2\,m_{2} 
\end{array}\right]
\\
B = \left[\begin{array}{c} -l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2-2\,\mathrm{\dot{\theta}}_{1}\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+g\,l_{1}\,\cos\left(\theta _{1}\right)\,\left(m_{1}+m_{2}\right)+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\\ l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right) \end{array}\right]
\\
C = \left[\begin{array}{c} \tau _{1}\\ \tau _{2} \end{array}\right]
\end{equation}
$$
所以有  
$$
\begin{equation}
\left[\begin{array}{c} \ddot{\theta} _{1}\\ \ddot{\theta} _{2} \end{array}\right]=
A^{-1}(C-B)
\end{equation}
$$
利用MATLAB求解上述矩阵运算，解得  
$$
\begin{equation}
\left[\begin{array}{c} & \frac{l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2+2\,\mathrm{\dot{\theta}}_{1}\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+\tau _{1}-g\,l_{1}\,\cos\left(\theta _{1}\right)\,\left(m_{1}+m_{2}\right)-g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)}{{l_{1}}^2\,m_{1}+{l_{1}}^2\,m_{2}-{l_{1}}^2\,m_{2}\,{\cos\left(\theta _{2}\right)}^2}+\\ & \frac{\left(l_{2}+l_{1}\,\cos\left(\theta _{2}\right)\right)\,\left(l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2-\tau _{2}+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\right)}{{l_{1}}^2\,l_{2}\,m_{1}+{l_{1}}^2\,l_{2}\,m_{2}-{l_{1}}^2\,l_{2}\,m_{2}\,{\cos\left(\theta _{2}\right)}^2}\\
&\\ &-\frac{\left(l_{2}+l_{1}\,\cos\left(\theta _{2}\right)\right)\,\left(l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2+2\,\mathrm{\dot{\theta}}_{1}\,l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+\tau _{1}-g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)-g\,l_{1}\,m_{1}\,\cos\left(\theta _{1}\right)-g\,l_{1}\,m_{2}\,\cos\left(\theta _{1}\right)\right)}{{l_{1}}^2\,l_{2}\,\left(-m_{2}\,{\cos\left(\theta _{2}\right)}^2+m_{1}+m_{2}\right)}-\\&\frac{\left(l_{1}\,l_{2}\,m_{2}\,\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2-\tau _{2}+g\,l_{2}\,m_{2}\,\cos\left(\theta _{1}+\theta _{2}\right)\right)\,\left({l_{1}}^2\,m_{1}+{l_{1}}^2\,m_{2}+{l_{2}}^2\,m_{2}+2\,l_{1}\,l_{2}\,m_{2}\,\cos\left(\theta _{2}\right)\right)}{{l_{1}}^2\,{l_{2}}^2\,m_{2}\,\left(-m_{2}\,{\cos\left(\theta _{2}\right)}^2+m_{1}+m_{2}\right)} \end{array}\right]
\end{equation}
$$
将数值带入得  
$$
\left[\begin{array}{c} -\frac{\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2+2\,\mathrm{\dot{\theta}}_{1}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+\tau _{1}-10\,\cos\left(\theta _{1}+\theta _{2}\right)-20\,\cos\left(\theta _{1}\right)}{{\cos\left(\theta _{2}\right)}^2-2}-\frac{\left(\cos\left(\theta _{2}\right)+1\right)\,\left(\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2-\tau _{2}+10\,\cos\left(\theta _{1}+\theta _{2}\right)\right)}{{\cos\left(\theta _{2}\right)}^2-2}\\ \frac{\left(2\,\cos\left(\theta _{2}\right)+3\right)\,\left(\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{1}}^2-\tau _{2}+10\,\cos\left(\theta _{1}+\theta _{2}\right)\right)}{{\cos\left(\theta _{2}\right)}^2-2}+\frac{\left(\cos\left(\theta _{2}\right)+1\right)\,\left(\sin\left(\theta _{2}\right)\,{\mathrm{\dot{\theta}}_{2}}^2+2\,\mathrm{\dot{\theta}}_{1}\,\sin\left(\theta _{2}\right)\,\mathrm{\dot{\theta}}_{2}+\tau _{1}-10\,\cos\left(\theta _{1}+\theta _{2}\right)-20\,\cos\left(\theta _{1}\right)\right)}{{\cos\left(\theta _{2}\right)}^2-2} \end{array}\right]
$$


这样就将$ {\ddot\theta}_{1} $和${\ddot\theta}_{2}$转化成了只与${\dot\theta}_{1}$,${\dot\theta}_{2}$,${\theta}_{1}$,${\theta}_{2}$,$\tau_{1}$,$\tau_{2}$有关的函数，之后在simulink中画方块图进行仿真即可。从结果发现，上述两个表达式具有高度对称性，因此虽然项数较多，但仿真过程并不难实现。  



在用simulink画方块图时，注意同一项因子最好只出现一次，例如仿真结果中所有的cos($\theta_1$+$\theta_2$)都用同一个方块引出，而不是同时存在多个引出为cos($\theta_1$+$\theta_2$)的方块。  
