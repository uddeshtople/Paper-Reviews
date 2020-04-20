# Guaranteed computation of robot trajectories 

Author: Uddesh Tople

###### tags : Constraint Satisfaction Problem (CSP), tubes. trajectory, contractor, constraints, estimation

## 1. Introduction
This article introduces a method for guaranteed integration of state equations. It provides reliable framework for the computation of robot trajectories with constrained equations. The framework first formalizes the problem and then apply constraints to the set of trajectories.

## 2. Constraint Propagation over Trajectories

### 2.1 Constraint Satisfaction problem(CSP)

 **CSP :-** The problems in which variables must satisfy some set of constraints.

**Contractors :-** A constraint $L$ can be applied on a box $[x]$ $\epsilon$ $\mathbb{R}$ with the help of contactor $C$.

**Decomposition** :- Problems involving complex equation can be broken into a set of primitive equations which cannot be broken further.
e.g. $\dot{x}$(.) . $\ddot{x}$(.) = $x$(.)  can be decomposed as follows:-
$\dot{x}$ = $b$
$\dot{b}$ = $a$
$x$ = $a$ . $b$

### 2.2 *Tubes* : envelopes for feasible trajectories

### Definition:-
A tube $([x])$ is an envelop enclosing an uncertain trajectory $x(.)$ : $\mathbb{R}$ $\rightarrow$ $\mathbb{R}^n$. A tube is an interval lying between two layers ($x^-$(.) $\&$ $x^+$(.)). where,
$x^-$(.) is lower bound and $x^+$(.) is upper bound for the trajectory. 
![](https://i.imgur.com/6L4P47b.png)

### Arithmetics on tube
Let, $\diamond$ $\epsilon$ {+,-,*,/} 
* Then, [x] $\diamond$ [y] is defined as the smallest tube having all feasible [x] (.) $\diamond$ [y] (.) solutions such that x(.) $\epsilon$ [x] (.) and y(.) $\epsilon$ [y] (.).
* Elementry function (e.g. sin,cos,etc) can be defined as the smallest tube with all feasible values of the elementary function.

### Integrals of tubes
Integral over a range t1 to t2 is defined as:-
![](https://i.imgur.com/wekyRIO.png)

which can also be deduced as
![](https://i.imgur.com/7vaEtui.png)

### Contractors for tube
A contractor applied on tube aims at removing the unfeasible trajectories according to the given constraint $L$

$[a](.)$ $\xrightarrow[]{C_L}$ $[b](.)$

The output of contractor $C_L$ is the tube $[b](.)$ such that:-

$\forall$ t, $[b](t)$ $\subseteq$ $[a](t)$,



$\binom{L(a(.))}{a(.)\epsilon[a](.)}$ $\Rightarrow$ a(.) $\epsilon$ $[b](.)$

### Implementation

Suppose a given tube is divided into k different slices of time sample $\delta$ each
The $k^{th}$ slice of the tube $[x](.)$ denoted by $[x](k)$ is given by $[k\delta,k\delta + \delta]$ x $[x](t_k)$ with $t_k$ $\epsilon$ $[k\delta,k\delta + \delta]$. The resultant tube enclose $[x^-(.),x^+(.)]$ inside an interval of step functions $[\underline{x^-}(.),\bar{x^+}(.)]$ such that:

$\forall$ t, $\underline{x^-}(t)$  $\underline{<}$ $x^-$(t) $\underline{<}$ $x^+$(t) $\underline{<}$ $\bar{x^+}(.)$

![](https://i.imgur.com/PeiMm9r.png)

## 3. Reliable Integration

### 3.1 CSP approach
$\dot{x}(t)$ = $f(x(t),t)$ with intitial condition $x(0)$ given
By decomposition, problem reduce to
**Variables** : $x(.),v(.)$
**Constraints** : 1. $\dot{x}$ = $v$
2. $v$ = $f(x,t)$
**Domains :** $[x](.),[v](.)$ 

The second constraint can be provided with the elementary contractors such as $C_-$ and $C_{sin}$. However, the contractor for the first constraint $(C_{\frac{d}{dt}})$ needs to be calculated.

### 3.2 Differential tube contractor
The differential contractor implies contraction that can be propogated along the whole domain in both forward $(C^{\rightarrow}_{\frac{d}{dt}})$ and backward $(C^{\leftarrow}_{\frac{d}{dt}})$ way.
$C$ = $C^{\rightarrow}_{\frac{d}{dt}}$ $\diamond$ $C^{\leftarrow}_{\frac{d}{dt}}$

#### Forward Contractor
A recurrence relation is encountered while solving equation $\dot{x}$=$v$, numerically:
$x(t+dt)=x(t)+dt.v(t)$

The new corresponding contractor with bounded values and intersections is:
$[x](t+dt)=[x](t+dt)$ $\cap$ $([x](t)+dt.[v](t))$

$[x](k+1)=[x](k+1)$ $\cap$ $([x](k)$ $\cap$ $[x](k+1)+[0,\delta].[v](k+1))$

#### Backward Contractor
The same  approach is applied for the backward contractor in backward for $(k-1)^{th}$ slice:
$[x](k-1)=[x](k-1)$ $\cap$ $([x](k)$ $\cap$ $[x](k-1)-[0,\delta].[v](k-1))$

![](https://i.imgur.com/xnyf9tz.png)
![](https://i.imgur.com/rZP0yhM.png)

### 3.3 Fixed points
Consider the problem with cyclic constraint as shown in fig below.
Due to the existence of one loop, an iterative resolution has to be processed until a fixed point is reached.
Though this method is applicable for this type of problem but it provide poor result. 
![](https://i.imgur.com/VxV6Aoz.png)

Note:- For simulation example, please refer paper.

## 4. Extension to State Estimation
A state estimation problem involving state constraints can be cast into a constraint network with four constraints:-

The *forward* propagation (1), the *backward* propagation (2), the *correction* (3) and the *state constraints* (4). In the literature, (1) is known as the *prediction*, (1)+(2) the *integration*, (3) the *correction*, (1)+(3)
the *filter* and (1)+(2)+(3) the *smoother*.

### 4.1 Added corrections and state constraints
A state estimation relies on the following equations:

$\dot{x}(t)$ = $f(x(t),t)$,
$y(t)$ = $g(x(t))$

where $f$: $\mathbb{R}^n$ x $\mathbb{R}$ $\rightarrow$ $\mathbb{R}^n$ is evolution function and $g$ : $\mathbb{R}^n$ $\rightarrow$ $\mathbb{R}^m$ is the observation function. $g$ allows correction over the state $x(t)$ from a given measurement $y(t)$ that can be bounded by $[y](t)$. This constraint is easily applied by performing a local contraction of the tube
$[x](.)$ at time $t$ such that:

$[x](t)=[x](t) \cap g^{-1}([y](t))$

A filter or a smoother procedure is then respectively obtained.

## 5. Conclusion
In this article, the principle is to model the problem as a constraint network and generate a contractor from each constraint. The contractors contract all tubes as much as possible up to a fixed point. Its use is well suited in high order differential contexts, non-linear equations, fleeting observations and real datasets.
