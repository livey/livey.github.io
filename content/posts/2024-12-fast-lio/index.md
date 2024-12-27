---
title: "Fast LIO Paper Reading"
date: 2024-12-27
# weight: 1
# aliases: ["/first"]
tags: ["SLAM", "LIO", "Paper Reading", "Fast LIO", "Signal Processing"]
author: "Fuwei Li"
# author: ["Me", "You"] # multiple authors
showToc: true
TocOpen: false
draft: false
hidemeta: false
comments: false
description: "Try to reveal the details of Fast LIO paper."
canonicalURL: "https://livey.github.io"
disableHLJS: true # to disable highlightjs
disableShare: false
disableHLJS: false
hideSummary: false
searchHidden: true
ShowReadingTime: true
ShowBreadCrumbs: true
ShowPostNavLinks: true
ShowWordCount: true
ShowRssButtonInSectionTermList: true
UseHugoToc: false
cover:
    image: "<image path/url>" # image path/url
    alt: "<alt text>" # alt text
    caption: "<text>" # display caption under cover
    relative: true # when using page bundles set this to true
    hidden: true # only hide on current single page
editPost:
    URL: "https://github.com/livey/livey.github.io/issues/new"
    Text: "Suggest Changes"
    appendFilePath: false
---

In this document, we complement some detailed derivatives in \[2].

# Details of the derivation

## Discrete model

Based on the $\boxplus$ operation defined above, we can discretize the continuous model in (1) at the IMU sampling period $\Delta t$ using a zero-order holder. The resultant discrete model is

$$ \mathbf{x}_{i+1} = \mathbf{x}_i \boxplus (\Delta t f(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i))$$

where $i$ is the index of IMU measurements, the function $f$, state $\mathbf{x}$, input $\mathbf{u}$, and noise $\mathbf{w}$ are defined as below:

$$
\mathcal{M} = SO(3) \times \mathbb{R}^{15}, \quad \text{dim}(\mathcal{M}) = 18
$$

where 

$$
\mathbf{x} \doteq \left[ \begin{matrix}
\mathbf{R}^T & \mathbf{P}^T  & \mathbf{v}^T & \mathbf{b}_\omega^T & \mathbf{b}_a^T & \mathbf{g}^T 
\end{matrix} 
\right]^T \in \mathcal{M}
$$

$$
\mathbf{u} \doteq \left[ \begin{matrix}
\boldsymbol{\omega}_m^T & \mathbf{a}_m^T
\end{matrix} \right]^T
$$

$$\mathbf{w} \doteq 
\left[ 
\begin{matrix}
\mathbf{n}_\omega^T &
 \mathbf{n}_a^T&
 \mathbf{n}_{b\omega}^T & \mathbf{n}_{ba}^T
\end{matrix}
 \right]^T
 $$
 
and

$$f(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i) = \left[ \begin{matrix}
\boldsymbol{\omega}_{m_i} - \mathbf{b}_{\omega_i} - \mathbf{n}_{\omega_i} \\
\mathbf{v}_i \\
\mathbf{R}_i (\mathbf{a}_{m_i} - \mathbf{b}_{a_i} - \mathbf{n}_{a_i}) +  \mathbf{g}_i \\
\mathbf{n}_{b\omega_i} \\
\mathbf{n}_{ba_i} \\
\mathbf{0}_{3 \times 1}
\end{matrix} \right]
$$



## Error propagation

$$ 
\hat{\mathbf{x}}_{i+1} = \hat{\mathbf{x}}_i \boxplus (\Delta t f(\hat{\mathbf{x}}_i, \mathbf{u}_i, \mathbf{0})) ; \quad \hat{\mathbf{x}}_0 = \bar{\mathbf{x}}_{k-1}.
$$

where $\hat{\mathbf{x}}_{i}$ is the predicted state and $\mathbf{u}_i$ is the control variable and $\mathbf{x}_i$ is the groundtruth.

and $\Delta t = \tau_{i+1} - \tau_i$. To propagate the covariance, we use the error state dynamic model obtained below:

$$
\begin{aligned}
\tilde{\mathbf{x}}_{i+1} &= \mathbf{x}_{i+1} \boxminus \hat{\mathbf{x}}_{i+1} \\
 &= (\mathbf{x}_i \boxplus \Delta t f(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i)) \boxminus (\hat{\mathbf{x}}_i \boxplus \Delta t f(\hat{\mathbf{x}}_i, \mathbf{u}_i, \mathbf{0})) \\
&\simeq \mathbf{F}_{\tilde{\mathbf{x}}} \tilde{\mathbf{x}}_i + \mathbf{F}_w \mathbf{w}_i
\end{aligned}
$$

where

$$
\mathbf{F}_{\tilde{\mathbf{x}}} = \begin{bmatrix}
 \text{Exp}(-\hat{\boldsymbol{\omega}}_i \Delta t) & 0  & 0 & -\mathbf{A}(\hat{\boldsymbol\omega}_i \Delta t)^T \Delta t & 0 & 0  \\
 0 & \mathbf{I} & \mathbf{I} \Delta t & 0 & 0 & 0  \\
 {\hat{\mathbf{R}}}_{i} \lfloor \hat{\mathbf{a}}_i \rfloor_\wedge \Delta t & 0 & \mathbf{I} & 0 & -\hat{\mathbf{R}}_{i} \Delta t & \mathbf{I} \Delta t \\
 0 &  0 & 0 & \mathbf{I} & 0 & 0 \\
 0 &  0 & 0 & 0 & \mathbf{I} & 0 \\
 0 &  0 & 0 & 0 & 0 & \mathbf{I}
\end{bmatrix}
$$

and

$$\mathbf{F}_w = \begin{bmatrix}
 -\mathbf{A}(\hat{\boldsymbol\omega}_i \Delta t)^T \Delta t & 0 & 0 & 0 \\
 0 & 0 & 0 & 0 \\
 0 & -\hat{\mathbf{R}}_{i} \Delta t & 0 & 0\\
 0 & 0 & \mathbf{I} \Delta t & 0 \\
 0 & 0 & 0 & \mathbf{I} \Delta t \\
 0 & 0 & 0 & 0
\end{bmatrix}
$$

## Derivative of the error propagation

Recall $\mathbf{x}_i = \hat{\mathbf{x}}_i \boxplus \tilde{\mathbf{x}}_i$, denote $g(\tilde{\mathbf{x}}_i, \mathbf{w}_i) = f(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i) \Delta t = f(\hat{\mathbf{x}}_i \boxplus \tilde{\mathbf{x}}_i, \mathbf{u}_i, \mathbf{w}_i) \Delta t$. Then the error state is

$$
\begin{equation}
 \tilde{\mathbf{x}}_{i+1} = 
\underbrace{\left( (\hat{\mathbf{x}}_i \boxplus \tilde{\mathbf{x}}_i) \boxplus g(\tilde{\mathbf{x}}_i, \mathbf{w}_i) \right) \boxminus \left( \hat{\mathbf{x}}_i \boxplus g(\mathbf{0}, \mathbf{0}) \right)}_{\mathbf{G}(\tilde{\mathbf{x}}_i, g(\tilde{\mathbf{x}}_i, \mathbf{w}_i))}
\end{equation}
$$



$$
\begin{aligned}
g(\tilde{\mathbf{x}}_i, \mathbf{w}_i) = 
\Delta{t}
\begin{bmatrix}
\boldsymbol{\omega}_i - (\hat{\mathbf{b}}_{\omega_i}+\tilde{\mathbf{b}}_{\omega_i})-\mathbf{n}_{\omega_i}\\
\hat{\mathbf{v}}_i + \tilde{\mathbf{v}}_i \\
(\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i)\left(\mathbf{a}_i - (\hat{\mathbf{b}}_{a_i}+\tilde{\mathbf{b}}_{a_i}) -\mathbf{n}_{a_i} \right) + (\hat{\mathbf{g}}_i + \tilde{\mathbf{g}}_i)\\
\mathbf{n}_{b\omega_i} \\
\mathbf{n}_{ba_i} \\
\mathbf{0}
\end{bmatrix} 
\end{aligned}$$

and

$$g(\mathbf{0}, \mathbf{0}) = 
\Delta t
\begin{bmatrix}
\boldsymbol{\omega}_i - \hat{\boldsymbol{b}}_{\omega_i} \\
\hat{\mathbf{v}}_i \\
\hat{\mathbf{R}}_i(\mathbf{a}_i -\hat{\mathbf{b}}_{a_i}) + \hat{\mathbf{g}}_i \\
\mathbf{0} \\
\mathbf{0} \\
\mathbf{0} 
\end{bmatrix}
$$

Following the chain rule of partial differentiation, the matrix $\mathbf{F}_{\tilde{\mathbf{x}}}$ and $\mathbf{F}_w$ in (5) are computed as below.

$$
\mathbf{F}_{\tilde{\mathbf{x}}} = \left( \frac{\partial \mathbf{G}(\tilde{\mathbf{x}}_i, g(\mathbf{0}, \mathbf{0}))}{\partial \tilde{\mathbf{x}}_i} + \frac{\partial \mathbf{G}(0, g(\tilde{\mathbf{x}}_i, 0))}{\partial \mathbf{g}(\tilde{\mathbf{x}}_i, 0)} \frac{\partial g(\tilde{\mathbf{x}}_i, 0)}{\partial \tilde{\mathbf{x}}_i} \right) \bigg|_{\tilde{\mathbf{x}}_i = 0}
$$

$$ \mathbf{F}_w = \left( \frac{\partial \mathbf{G}(0, g(0, \mathbf{w}_i))}{\partial g(0, \mathbf{w}_i)} \frac{\partial g(0, \mathbf{w}_i)}{\partial \mathbf{w}_i} \right) \bigg|_{\mathbf{w}_i = 0}
$$

### How to get $\mathbf{F}_{\tilde{\mathbf{x}}}$ and $\mathbf{F}_w$

$$ \tilde{\mathbf{x}}_{i+1} = 
\underbrace{
( \underbrace{(\hat{\mathbf{x}}_i \boxplus \tilde{\mathbf{x}}_i)}_{A_1} \boxplus 
g(\tilde{\mathbf{x}}_i, \mathbf{w}_i))}_{A}
\boxminus 
\underbrace{(\hat{\mathbf{x}}_i \boxplus g(\mathbf{0}, \mathbf{0}))}_{B}$$

So, we have

$$
\begin{aligned}
\frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial \tilde{\mathbf{x}}_i} 
&= \frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial{A}}\frac{\partial{A}}{\partial \tilde{\mathbf{x}}_i} \\
&= \frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial{A}}
\left(\frac{\partial{A}}{\partial{A}_1}\frac{\partial{A}_1}{\partial\tilde{\mathbf{x}}_i} + \frac{\partial{A}}{\partial{g}(\tilde{\mathbf{x}}_i, \mathbf{w}_i)}\frac{\partial{g}}{\partial{\tilde{\mathbf{x}}}_i}
\right)
\end{aligned}
$$

According to Appendix B \[2], for $Q, R$ in $SO(3)$, $\mathbf{J}_{Q}^{Q-R} = \mathbf{J}^{-1}_r(\boldsymbol{\theta})$, where $\boldsymbol{\theta} = Q \boxminus R$,  $\boldsymbol{\theta} = \theta \mathbf{u}$,

$\mathbf{J}_r^{-1}(\boldsymbol\theta) = \mathbf{I} + \frac{1}{2} [\boldsymbol\theta]_{\times} + \left( \frac{1}{\theta^2} - \frac{1 + \cos \theta}{2 \theta \sin \theta} \right) [\boldsymbol\theta]_{\times}^2$, 

$$[\boldsymbol{\theta}]_{\times} = \begin{bmatrix} 
0 & -\theta_z & \theta_y \\ 
\theta_z & 0 & -\theta_x \\
 -\theta_y & \theta_x & 0 
\end{bmatrix}.$$

And for $P, Q \in\mathbb{R}^n$, $\frac{\partial{P\boxminus Q}}{\partial{P}} = \mathbf{I}$ and $\frac{\partial{P\boxminus Q}}{\partial{Q}} = -\mathbf{I}$,

Then we have

$$
\frac{\partial{\tilde{\mathbf{x}}}_{i+1}}{\partial{A}}\bigg|_{\mathbf{w}_i=0, \tilde{\mathbf{x}}_i =0} = \mathbf{I}
$$

For $Q, R$ in $SO(3)$, $\mathbf{J}_{Q}^{Q\boxplus \boldsymbol{\theta}} = \mathbf{R}(\boldsymbol{\theta})^T$, where $\mathbf{R}(\boldsymbol{\theta}) = \text{Exp}(\theta \mathbf{u}) 
\triangleq 
\mathbf{I} + \sin\theta[\mathbf{u}]_\times + (1-\cos\theta)[\mathbf{u}]_\times^2$. And for $P, Q \in\mathbb{R}^n$, $\frac{\partial{P\boxplus Q}}{\partial{P}} = \mathbf{I}$.

So,

$$
\begin{aligned}
\frac{\partial{A}}{\partial{A}_1}\bigg|_{\boldsymbol{w}=0, \tilde{\mathbf{x}}_i=0} 
&= 
\begin{bmatrix}
\mathbf{R}^T(\Delta{t}(\boldsymbol{\omega}_i - \hat{\mathbf{b}}_{\omega_i}) & 0 & 0 &0 &0 &0 \\
0 &\mathbf{I} &0 &0 &0 &0 \\
0  &0 &\mathbf{I} &0 &0 &0 \\
0  &0 &0 &\mathbf{I} &0 &0 \\
0  &0 &0 &0 &\mathbf{I} &0 \\
0  &0 &0 &0 &0 &\mathbf{I} 
\end{bmatrix}\\
&\overset{(a)}{=}
\begin{bmatrix}
\mathbf{R}(-\Delta{t}(\boldsymbol{\omega}_i - \hat{\mathbf{b}}_{\omega_i}) & 0 & 0 &0 &0 &0 \\
0 &\mathbf{I} &0 &0 &0 &0 \\
0  &0 &\mathbf{I} &0 &0 &0 \\
0  &0 &0 &\mathbf{I} &0 &0 \\
0  &0 &0 &0 &\mathbf{I} &0 \\
0  &0 &0 &0 &0 &\mathbf{I} 
\end{bmatrix}
\end{aligned}
$$

where equation (a) establishes because we note that $[\mathbf{u}]_\times $is a skew-symmetric matrix. Then, we have $[\mathbf{u}]_\times^T = [-\mathbf{u}]_\times$. Thus, we get $\mathbf{R}^T(\boldsymbol{\theta}) = \mathbf{R}(\boldsymbol{-\theta})$.

For $Q$ in $SO(3)$, $\mathbf{J}_{\boldsymbol{\theta}}^{Q\boxplus \boldsymbol{\theta}} = \mathbf{J}_r(\boldsymbol{\theta}) \triangleq \mathbf{I} - \frac{1 - \cos \theta}{\theta^2} [\theta]_{\times} + \frac{\theta - \sin \theta}{\theta^3} [\theta]_{\times}^2$; and for $\mathbf{x}, \mathbf{y} \in\mathbb{R}^n$, $\frac{\partial \mathbf{x} + \mathbf{y}}{\partial\mathbf{y}} = \frac{\partial \mathbf{x} + \mathbf{y}}{\partial\mathbf{x}} = \mathbf{I}$


$$
\frac{\partial{A}_1} {\partial{\tilde{\mathbf{x}}}_{i}}\bigg|_{\tilde{\mathbf{x}}_i =0} =
\begin{bmatrix}
\mathbf{J}_r(\tilde{\boldsymbol\theta}_i)|_{\tilde{\boldsymbol{\theta}}_i=0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I}  &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I}  &\mathbf{0} \\ 
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} 
\end{bmatrix} = \mathbf{I}
$$

$$
\frac{\partial{A}}{\partial{g}}\bigg|_{\tilde{\mathbf{x}}_i=0, \mathbf{w}_i=0} = 
\begin{bmatrix}
\mathbf{J}(\Delta{t}(\boldsymbol{\omega}_i-\hat{\mathbf{b}}_{w_i})) &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I}\\
\end{bmatrix}
$$



$$
\frac{\partial g}{\partial{\tilde{\mathbf{x}}}_i} =\Delta{t}
\begin{bmatrix}
\mathbf{0} &\mathbf{0} &\mathbf{0} &-\mathbf{I} &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
-(\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i)\left[\mathbf{a}_i - (\hat{\mathbf{b}}_{a_i}+\tilde{\mathbf{b}}_{a_i}) -\mathbf{n}_{a_i} \right]_\times\mathbf{J}(\tilde{\boldsymbol{\theta}})&\mathbf{0} &\mathbf{0} &\mathbf{0} & -\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i &\mathbf{I} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} & \mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0}  & \mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0}  & \mathbf{0} &\mathbf{0} &\mathbf{0}
\end{bmatrix}
$$

where we use $\frac{\partial\left((\mathbf{R}\boxplus\boldsymbol{\theta})\mathbf{v}\right)}{\partial\boldsymbol{\theta}} = \frac{\partial(\mathbf{R}\boxplus\boldsymbol{\theta})\mathbf{v}}{\partial(\mathbf{R}\boxplus\boldsymbol{\theta})}\frac{\partial\mathbf{R}\boxplus\boldsymbol{\theta}}{\partial\boldsymbol{\theta}} = -(\mathbf{R}\boxplus\boldsymbol{\theta})[\mathbf{v}]_\times \mathbf{J}(\boldsymbol{\theta})$.

So,

$$
\frac{\partial{A}}{\partial{\tilde{\mathbf{x}}_i}} = \frac{\partial{A}}{\partial{g}}\frac{\partial{g}}{\partial{\tilde{\mathbf{x}}_i}} =
\Delta{t}
\begin{bmatrix}
\mathbf{0} &\mathbf{0} &\mathbf{0} &-\mathbf{J}(\Delta{t}(\boldsymbol{\omega}_i-\hat{\mathbf{b}}_{w_i})) &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
-(\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i)\left[\mathbf{a}_i - (\hat{\mathbf{b}}_{a_i}+\tilde{\mathbf{b}}_{a_i}) -\mathbf{n}_{a_i} \right]_\times\mathbf{J}(\tilde{\boldsymbol{\theta}}) &\mathbf{0} &\mathbf{0} &\mathbf{0} &-\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i &\mathbf{I}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}
\end{bmatrix}
$$

Finally, we have

$$
\begin{aligned}
\frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial \tilde{\mathbf{x}}_i} 
&= \frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial{A}}\frac{\partial{A}}{\partial \tilde{\mathbf{x}}_i} \\
&= \frac{\partial \tilde{\mathbf{x}}_{i+1}}{\partial{A}}
\left(\frac{\partial{A}}{\partial{A}_1}\frac{\partial{A}_1}{\partial\tilde{\mathbf{x}}_i} + \frac{\partial{A}}{\partial{g}(\tilde{\mathbf{x}}_i, \mathbf{w}_i)}\frac{\partial{g}}{\partial{\tilde{\mathbf{x}}}_i}
\right) \\
&=  
\begin{bmatrix}
\mathbf{R}(-\Delta{t}(\boldsymbol{\omega}_i - \hat{\mathbf{b}}_{\omega_i}) &\mathbf{0} &\mathbf{0} &-\mathbf{J}(\Delta{t}(\boldsymbol{\omega}_i-\hat{\mathbf{b}}_{w_i}))\Delta{t}  &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{I} &\Delta{t}\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
-(\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i)\left[\mathbf{a}_i - (\hat{\mathbf{b}}_{a_i}+\tilde{\mathbf{b}}_{a_i}) -\mathbf{n}_{a_i} \right]_\times\mathbf{J}(\tilde{\boldsymbol{\theta}})\Delta{t}  &\mathbf{0} &\mathbf{I} &\mathbf{0} &-(\mathbf{R}_i\boxplus\tilde{\boldsymbol{\theta}}_i)\Delta{t} &\Delta{t}\mathbf{I} \\
\mathbf{0}  &\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} &\mathbf{0} \\
\mathbf{0}  &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} \\
\mathbf{0}  &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} 
\end{bmatrix}
\end{aligned}
$$

$$
\frac{\partial g}{\partial{\mathbf{w}}_i} =
\Delta{t}
\begin{bmatrix}
-\mathbf{I} &\mathbf{0} &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &-(\hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i)  &\mathbf{0} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I} \\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0} 
\end{bmatrix}
$$



Then, we have

$$
\frac{\partial\tilde{\mathbf{x}}_{i+1}}{\partial{\mathbf{w}}_i}  = \frac{\partial\tilde{\mathbf{x}}_{i+1}}{\partial{A}}\frac{\partial{A}}{\partial{g}}\frac{\partial{g}}{\partial{\mathbf{w}}_i}  = 
\Delta{t}
\begin{bmatrix}
-\mathbf{J}(\Delta{t}(\boldsymbol{\omega}_i-\hat{\mathbf{b}}_{w_i})) &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}\\
\mathbf{0} & - \hat{\mathbf{R}}_i\boxplus\tilde{\boldsymbol{\theta}}_i &\mathbf{0} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{I} &\mathbf{0}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{I}\\
\mathbf{0} &\mathbf{0} &\mathbf{0} &\mathbf{0}
\end{bmatrix}
$$

In \[2], it defines $A(\phi)^{-1}$, however, we need $A(\phi)$ actually. In \[3], it defines

$$
A(\psi) = I + \left( \frac{1 - \cos \| \psi \|}{\| \psi \|} \right) \frac{\hat{\psi}}{\| \psi \|} + \left( 1-\frac{ \sin \| \psi \|}{\| \psi \|} \right) \frac{\hat{\psi}^2}{\| \psi \|^2}
$$

Compare it with our definition of $\mathbf{J}(\boldsymbol{\theta})$ and note that $A(\phi)^\top = A(-\phi)$. We conclude that they are exactly the same.

In summary, we have given the detailed derivatives of the state propagation equation.

# References

> \[1] J. Solà, J. Deray, and D. Atchuthan, “A micro Lie theory for state estimation in robotics.” arXiv, Dec. 08, 2021. doi: 10.48550/arXiv.1812.01537.

> \[2] W. Xu and F. Zhang, “FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter.” arXiv, Apr. 14, 2021. Accessed: May 06, 2024. \[Online]. Available: http://arxiv.org/abs/2010.08196

> \[3] F. Bullo and R. M. Murray, “Proportional derivative (pd) control on the euclidean group,” in European control conference, vol. 2, 1995, pp. 1091–1097.


