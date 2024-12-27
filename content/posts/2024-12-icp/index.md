---
title: "Iterative Closest Point Problem"
date: 2024-12-26
# weight: 1
# aliases: ["/first"]
tags: ["Iterative Closest Point", "ICP", "Signal Processing", "Optimization", "SLAM"]
author: "Fuwei Li"
# author: ["Me", "You"] # multiple authors
showToc: true
TocOpen: false
draft: false
hidemeta: false
comments: false
description: "A detailed derivation of the Iterative Closest Point (ICP) problem."
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

# Problem Formulation

Let two 3D point-sets $\mathcal{X} = \{\mathbf{x}_i\}, i = 1, \ldots, N$ and $\mathcal{Y} = \{\mathbf{y}_j\}, j = 1, \ldots, M$, where $\mathbf{x}_i, \mathbf{y}_j \in \mathbb{R}^3$ are point coordinates, be the data point-set and the model point-set respectively. The goal is to estimate a rigid motion with rotation $\mathbf{R} \in SO(3)$ and translation $\mathbf{t} \in \mathbb{R}^3$ that minimizes the following $L_2$-error $E$:

$$\underset{\mathbf{R}, \mathbf{t}}{\arg\min E(\mathbf{R}}, \mathbf{t}) = \sum_{i=1}^N e_i(\mathbf{R}, \mathbf{t})^2 = \sum_{i=1}^N \left\| \mathbf{R} \mathbf{x}_i + \mathbf{t} - \mathbf{y}_{j^*} \right\|^2 \tag{1}$$

where $e_i(\mathbf{R}, \mathbf{t})$ is the per-point residual error for $x_i$. Given $\mathbf{R}$ and $\mathbf{t}$, the point $y_{j^*} \in \mathcal{Y}$ is denoted as the optimal correspondence of $x_i$, which is the closest point to the transformed $x_i$ in $\mathcal{Y}$, i.e.,

$$j^* = \underset{j \in \{1, \ldots, M\}}{\arg \min} \left\| \mathbf{R} x_i + \mathbf{t} - \mathbf{y}_j \right\|\tag{2}$$

Note the short-hand notation used here: $j^*$ varies as a function of $(\mathbf{R}, \mathbf{t})$ and also depends on $x_i$.

Iterative closest point algorithm solves problem (1) by iteratively solving problem (1) and (2).

# Iterative Solution

## Solving problem (2) with fixed pose

With fixed pose, problem (2) can be solved with a computational complexity of $\mathcal{O}(mn)$. However, we can build an oct-tree to accelerate the closest point search.

## Solving problem (1) with known point correspondence

With known point correspondence, problem (1) can be solved analytically.

$$\min_{\mathbf{R}\in SO(3), \mathbf{t}}\|\mathbf{R}\mathbf{x}_i+t-\mathbf{y}_i\|_2^2$$

For any $\mathbf{R}$, taking the derivative of the objective with respect to $\mathbf{t}$ and letting it equal to zero we have

$$\mathbf{t} = \frac{1}{N}\sum_i(\mathbf{y}_i - \mathbf{R}\mathbf{x}_i) = \bar{\mathbf{y}} - \mathbf{R}\bar{\mathbf{x}}$$

where we have $\bar{y}\triangleq\frac{1}{N}\sum_i \mathbf{y}_i$ and $\bar{\mathbf{x}}\triangleq\frac{1}{N}\sum_i \mathbf{x}_i$. So, plug them into the objective function and let $\mathbf{x}_i\triangleq \mathbf{x}_i-\bar{\mathbf{x}}$ and $\mathbf{y}_i\triangleq \mathbf{y}_i - \bar{\mathbf{y}}$, we have

$$\min_{\mathbf{R}\in SO(3)} \|\mathbf{R}\mathbf{x}_i - \mathbf{y}_i\|_2^2$$

It is equivalent to

$$\max_{\mathbf{R}\in SO(3)}\,\mathbf{y}_i^\top\mathbf{R}\mathbf{x}_i$$

and further we have

$$\max_{\mathbf{R}\in SO(3)}\text{tr}(\mathbf{R}\sum_i \mathbf{x}_i\mathbf{y}_i^\top)$$

Let $\begin{matrix}\mathbf{M} = \sum_i \mathbf{x}_i\mathbf{y}_i^\top\end{matrix}$. We are thus solving:

$$\max_{\mathbf{R}\in SO(3)}\text{tr}(\mathbf{R}\mathbf{M})$$

Let the SVD of $\mathbf{M} = \mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top$. Then $\text{tr}(\mathbf{R}\mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top) = \text{tr}(\mathbf{V}^\top\mathbf{R}\mathbf{U}\mathbf{\Sigma}) \triangleq \text{tr}(\mathbf{Q}\mathbf{\Sigma})$, where $\mathbf{Q}=\mathbf{V}^\top\mathbf{R}\mathbf{U}$ is an orthonormal matrix.

$$\text{tr}(\mathbf{Q}\mathbf{\Sigma}) = \sum_i q_{i,i}\sigma_{i}\leq \sum_i \sigma_{i}$$

The last equality holds when $q_{i,i}=1$ for all $i$ and $\sigma_{i,i}>0$, which means $\mathbf{Q}=\mathbf{I}$. Therefore, we have $\mathbf{R}=\mathbf{V}\mathbf{U}^\top$.

Since $SO(3)$ puts a determinant constraint on the orthonormal matrix with $\text{det}(R)=1$, we have two cases:

1. If $\text{det}(\mathbf{R})=1$, the optimal solution is $\mathbf{R}=\mathbf{V}\mathbf{U}^\top$.
2. If $\text{det}(\mathbf{R})=-1$, the algorithm fails.

In the case where $\text{det}(\mathbf{R})=-1$ and there exists at least one $\sigma_i$ that is zero, we can multiply $-1$ to its corresponding column of $\mathbf{U}$ or $\mathbf{V}$.

In the case where $\text{det}(\mathbf{R})=-1$ and all $\sigma_i$'s are positive, "This can happen only when the noises are very large. In that case, the least-squares solution is probably useless anyway. A better approach would be to use a RANSAC-like technique (using 3 points at a time) to combat against outliers"\[1].

# References

> \[1] K. S. Arun, T. S. Huang, and S. D. Blostein, “Least-Squares Fitting of Two 3-D Point Sets,” IEEE Trans. Pattern Anal. Mach. Intell., vol. PAMI-9, no. 5, pp. 698–700, Sep. 1987, doi: 10.1109/TPAMI.1987.4767965.

> \[2] Y. Zheng, Y. Kuang, S. Sugimoto, K. Astrom, and M. Okutomi, “Revisiting the PnP Problem: A Fast, General and Optimal Solution,” in 2013 IEEE International Conference on Computer Vision, Sydney, Australia: IEEE, Dec. 2013, pp. 2344–2351. doi: 10.1109/ICCV.2013.291.

> \[3] G. Terzakis and M. Lourakis, “A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem,” in Computer Vision – ECCV 2020, vol. 12346, A. Vedaldi, H. Bischof, T. Brox, and J.-M. Frahm, Eds., in Lecture Notes in Computer Science, vol. 12346. , Cham: Springer International Publishing, 2020, pp. 478–494. doi: 10.1007/978-3-030-58452-8\_28.




