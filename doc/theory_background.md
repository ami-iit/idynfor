# iDynFor theory background

This document provide an overview of the different semantics between iDynTree and Pinocchio
data-structures and interfaces, and provide a reference for the implementation in iDynFor classes, that implements iDynTree-inspired interfaces using Pinocchio.


## iDynTree::Model vs pinocchio::Model

### Link and Joints

In `iDynTree::Model` a multibody is composed by its links that are interconnected by joints.
Each link is connected to another link by a joint, so you can't attach a joint to another joint
without having a link in the middle. Furthermore, `iDynTree::Model` consider all models to be
floating base by default, so there is no "joint" connecting the base to "universe" or "world" frame.
Only links have a frame w.r.t. to which you can compute forward kinematics or the jacobian, while there is no explicit concept of "joint frame"

On the other hand, in `pinocchio::Model` joints are interconnected to each other, to build
the so-called "kinematic tree". So multiple joints can be connected to each other, without having
a body in the middle. Furthermore, each link must have a parent joint, even if it is the first body of the kinematic tree.
So, an empty pinocchio model contains always the `universe` link, and the parent joint of the `universe` link.
The rest of the model can be connected to the `universe` reference frame with any kind of frame, but for consistency
with `iDynTree` the pinocchio models built in `iDynFor` are always connected to "universe" with a `pinocchio::JointFreeFlyer` (i.e. 6-DOF joint).

For these reasons, the count of links and joints in `iDynTree::Model` and  `pinocchio::Model` are different.

For links, in iDynTree only the internal bodies that compose the multibody model are counted, while for `pinocchio` also the `universe` "link" is considered.
For joints, in iDynTree only the joints that internconnect internal links are considered, while in `pinocchio` two additional joints are considerd:
* the joint that connects the floating base to `universe`,
* the  parent joint of `universe`.

As an example, if you have a single link floating base model, this is the number of its joints and links w.r.t. if modeled used `iDynTree::Model` and  `pinocchio::Model`:

| Quantity |  `iDynTree::Model`  | `pinocchio::Model` |
|:--------:|:---------------------:|:--------------------:|
| Number of Links | 1             |          2            |
| Number of Joints | 0             |         2            |

### Joints

#### iDynTree

In iDynTree, the joint information can be accessed via the `iDynTree::IJoint` C++ interface.
It is an interface that is designed to be `undirect`, i.e. it can explicitly specify at runtime
which link is the one considered `parent` and which one is considered `child`.

Assuming that the joint `J` connects to links `C` and `D`, and given that $s \in \mathbb{R}^{dof}$ is the vector
containing the position of internal dofs of the joint, the `iDynTree::IJoint::getTransform(s, C_index, D_index)` computes the transform ${}^C H_D$. How this transform is computed depends on the type of joint used.

##### `FixedJoint`

A fixed joint represents a constraint of all the 6 DOF between two links. So, it is parametrized directly by the transform ${}^C H_D$, and it returns ${}^C H_D$ if `getTransform(s, C_index, D_index)` is called or ${}^C H_D^{-1} = {}^D H_C$ if `getTransform(s, D_index, C_index)` is called.

##### `RevoluteJoint`

A revolute joint represents a constraint that constraints 5 DOF between two links, leaving the two links free to move around a given axis. So, the relative position between the two links is represented by the so-called joint position $s \in \mathbb{R}^{1}.

So, to represent a `RevoluteJoint` iDynTree uses the following parameters:
* The rest transform ${}^C H_D^{rest}$, that can be accessed via `iDynTree::getRestTransform(C_index, D_index)`, that is the relative pose between the two links when $s = 0$ .
* The axis along which the rotation is allowed, is itself represented by two vectors:
  * The axis direction $d^{axis} \in \mathbb{R}^3$, that being a pure direction is subject to the constraint $|d| = 1$,
  * The axis origin $^{axis} \in \mathbb{R}^3$, that is a point in the 3D space.

The transform between the frames $C$ and $D$, given the joint position $s$ is computed as in the following:

$$
{}^C H_D(s) = H({}^C d^{axis}, {}^C o^{axis}, s) {}^C H_D^{rest}
$$

The axis can be represented in both $C$ and $D$ frames, and you can go from one to another representation as in the following equations:

$$
{}^C d^{axis} = -{}^C R_D {}^D d^{axis}
$$

$$
{}^C o^{axis} = {}^C R_D {}^D o^{axis} + {}^C o_D
$$

Note that the $-$ in the axis transformation formula is required to ensure that:

$$
(H({}^C d^{axis}, {}^C o^{axis}, s) {}^C H_D^{rest})^{-1}  = (H({}^D d^{axis}, {}^D o^{axis}, s) {}^D H_C^{rest})
$$


#### Pinocchio

In pinocchio, a joint connecting a link $C$ and a link $D$ has its own frame, that we will call $J$.

##### FixedJoint

In Pinocchio, to represent a fixed joint there are two transforms:
* The transform ${}^C H_J$
* The transform ${}^J H_C$

For the specific case of the FixedJoint, there is no constraint on the location of the frame $J$.


##### RevoluteJoint

In Pinocchio, to represent a revolute joint there are two transforms and an axis
* The transform ${}^C H_J$
* The transform ${}^J H_D$
* The axis ${}^J d^{axis}$

And the total rotation is computed as:

$$
{}^C H_D (s) = {}^C H_J H({}^J d^{axis}, 0, s) {}^J H_D
$$

The major difference w.r.t. to iDynTree is that the axis has no notion of "origin", so for placing the frame $J$
we need to place it in the origin of the axis.

#### Convert from iDynTree to pinocchio

##### FixedJoint

We can simply place $J = D$, and so:

${}^C H_J = {}^C H_J^{rest}$
${}^J H_D = 1_4$

##### RevoluteJoint

We can simply set the origin of $J$ to match the origin of the axis and the orientation of $J$ to match the orientation of $D$, i.e. $J = (o^{axis},[D])$. Then we set ${}^J H_D$ such that the overall result match.

$$
{}^C H_J = {}^C H_{o^{axis} [D]} = {}^C H_D^{rest} {}^D H_{o^{axis} [D]} = {}^C H_D^{rest} \begin{bmatrix} I_3 & {}^D o^{axis} \\\\ 0_{3 \times 1}  & 1 \end{bmatrix}
$$

$$
{}^J H_D = {}^{o^{axis} [D]} H_D = {}^{o^{axis}[D]} H_D = \begin{bmatrix} I_3 & -{}^D o^{axis} \\\\ 0_{3 \times 1}  & 1 \end{bmatrix}
$$

$$
{}^J d^{axis} = {}^C d^{axis}
$$

### Model Position

**Note: as of iDynTree 8.1.0, iDynTree only supports internal joints of type revolute or prismatic, so
in this documentation we will always consider the case in which the  derivative of internal shape of the multibody model is used to represent the internal velocity of the multibody model, even if in theory
the Joint interface used by iDynTree could also support joints in which it may be convenient to use
as velocity a quantity different from the derivative of the joint position, such as spherical joints.**

In `iDynTree` the model position is represented by a pair of quantities:
* ${}^A H_B \in SE(3)$, i.e. the homogeneous transform between the base link frame ($B$) and the universe/world "absolute" frame ($A$),
* $s \in \mathbb{R}^{dof}$, i.e. vector of internal position, also called "shape" (hence the "s") of the multibody model.

In `pinocchio`, the model position is represented by a single quantity:
* $q \in \mathbb{R}^{nq}$, i.e. the vector of position of all joints, including the joint connecting the base to the "universe" frame. The first 7 elements are the position of the joint connecting the base to the "universe" frame, with the first 3 elements being the ${}^A o_B$, and the other 4 elements a quaternion
corresponding to ${}^A R_B$, see https://github.com/stack-of-tasks/pinocchio/issues/65#issuecomment-160931189 .

To convert from the `iDynTree` representation to the `pinocchio` one, one needs to convert the ${}^A H_B$ to the first 7 elements of $q$, and then convert $s$ to the last $dof=nq-7$ elements of $q$, accounting for the fact that the ordering used in iDynTree and in pinocchio is different.

For what regards the internal joint positions, for both Pinocchio and iDynTree they are represented by $dof$ scalars. For iDynTree, these are exactly the elements of the $s \in \mathbb{R}^{dof}$ vector, while for the Pinocchio they are the last $dof$ elements of the $q$ vectors. Anyhow, what changes is the
**serialization** of this elements. In the iDynTree case, the DOF serialization can be arbitrary. For example, when loading a model the desired serialization can be passed as the `consideredJoints` arguments of `iDynTree::ModelLoader::loadReducedModelFromFullModel` or `iDynTree::ModelLoader::loadReducedModelFromFile` methods. In the pinocchio case, the joint serialization **must** follow a strict depth-first order induced by the kinematic structure of the model an the selected floating base. To go from iDynTree's $s$ to Pinocchio's $q[7:end]$, we need to define an appropriate [permutation matrix](https://en.wikipedia.org/wiki/Permutation_matrix) $P \in \mathbb{R}^{dof \times dof}$:

$$
q[7:end] = P s
$$

The $P$ matrix can be computed by matching iDynTree's DOF serialization, obtained from the `iDynTree::IJoint::getPosCoordsOffset` and `iDynTree::IJoint::getDOFsOffset` methods and the Pinocchio's DOF serialization, obtained from  `pinocchio::ModelTpl::idx_qs` and `pinocchio::ModelTpl::idx_vs` attributes.

### Frame Velocity

Before discussing how iDynTree and pinocchio describe the velocity of a multi-body model,
we need to briefly discuss how the velocity of a frame attached to a rigid body can be represented.

Given two frames $A$ and $B$, the relative pose between this two frames is mathematically represented by an element ${}^A H_B \in SE(3)$ defined as:

$$
{}^A H_B =
\begin{bmatrix}
{}^A R_B & {}^A o_B \\
0_{3\times1} & 1
\end{bmatrix}
$$

This mathematical objects are represented in pinocchio by `pinocchio::SE3Tpl`  and in iDynTree by `iDynTree::Transform`. The time derivative of the pose ${}^A H_B$ is represented by ${}^A \dot{H}_B$, but for compactiness tipically alternative representation in the form of 6D vectors are used.

The one commonly used in robotics and multi-body dynamics are:


|           `iDynTree`          |     `Pinocchio`     |   Math      |
|:-----------------------------:|:-------------------:|:-----------:|
|      `MIXED_REPRESENTATION`     | `LOCAL_WORLD_ALIGNED` | ${}^{B[A]} \mathrm{v} = ({}^A \dot{o}_B, ({}^A \dot{R}_B {}^A R_B^T)^\vee)$ .  |
| `INERTIAL_FIXED_REPRESENTATION` |        `WORLD`        | ${}^{A} \mathrm{v} = ({}^A \dot{o}_B - ({}^A \dot{R}_B {}^A R_B^T) {}^A \dot{o}_B , ({}^A \dot{R}_B {}^A R_B^T)^\vee)$  . |
|  `BODY_FIXED_REPRESENTATION`   |        `LOCAL`        | ${}^{B} \mathrm{v} = ({}^A R_B^T {}^A \dot{o}_B , ({}^A \dot{R}_B {}^A R_B^T)^\vee)$ . |

Sources and derivation for the equations:
* `MIXED_REPRESENTATION` Eq. 35 in https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2
* `INERTIAL_FIXED_REPRESENTATION` Eq. 23 in https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2
* `BODY_FIXED_REPRESENTATION` Eq. 18 in https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2

Related Pinocchio issues:
* https://github.com/stack-of-tasks/pinocchio/issues/1624
* https://github.com/stack-of-tasks/pinocchio/issues/1336

Both iDynTree and pinocchio offer support for all the three velocity representations, even if in different ways.
iDynTree defaults to the `MIXED_REPRESENTATION`/`LOCAL_WORLD_ALIGNED`, while pinocchio to `BODY_FIXED_REPRESENTATION`/`LOCAL`.
Both iDynTree and pinocchio use the linear/angular velocity for serialization of 6D velocity, differently from Featherstone book "Rigid Body Dynamics Algorithms" that instead uses angular/linear.
Both iDynTree and pinocchio support changing the quantity representation used for getting frame velocity or jacobians, but the main different is that
for pinocchio the base velocity is always expressed in `LOCAL`, while in iDynTree when ones changes the representation used for frame velocity or jacobians,
it also changes the representation used for the base velocity.

### Model Velocity

Both in iDynTree and in pinocchio, the model velocity are represented by a $\mathbb{R}^{6+dof}$ vector, that we will call $\nu^{pin}$ and $\nu^{idyn}$.

To convert the base part of the velocity from iDynTree to pinocchio, we need to consider that in the case of pinocchio the base velocity is always expressed in   `BODY_FIXED_REPRESENTATION`/`LOCAL` (i.e. $B$), while in iDynTree is expressed in a different frame depending on the `FrameVelocityRepresentation`  set in `iDynTree::KinDynComputations`, i.e. $B[A]$ for  `MIXED_REPRESENTATION` , $A$ for  `INERTIAL_FIXED_REPRESENTATION` and $B$ for `BODY_FIXED_REPRESENTATION`. So if we call the frame representation used by iDynTree $W$ we have that:

$$
\nu^{pin}[1:6] = {}^B X_{velReprFrame} \nu^{idyn}[1:6]
$$

while for the internal joints, we need to remap the joints exactly with the same permutation matrix used at the position level:

$$
\nu^{pin}[7:end] = P \nu^{idyn}[7:end]
$$

So the complete transform matrix to go from iDynTree model velocity to pinocchio model velocity:

$$
\nu^{pin}
= \begin{bmatrix}
{}^B X_{velReprFrame} & 0_{6 \times dof} \\\\
0_{dof \times 6} & P
\end{bmatrix}
\nu^{idyn}
$$

For compactiness, in the rest of document we will indicate this matrix transorm as $$:

$$
{}^{pin} T {}^{idyn} =
\begin{bmatrix}
{}^B X_{velReprFrame} & 0_{6 \times dof} \\\\
0_{dof \times 6} & P
\end{bmatrix}
$$

When $T$ is used without any additional notation, we will be referring to ${}^{pin} T {}^{idyn}$.

### Jacobians

Pinocchio can compute out of the box Jacobians that when moltiplied by $\nu^{pin}$ return the 6D
velocity expressed with desired velocity representation. However, to transform this in Jacobians
that can be multiplied by $\nu^{idyn}$ we need to transform appropriately, i.e.:

$$
J^{pin} \nu^{pin} = J^{pin} \begin{bmatrix}
{}^B X_{velReprFrame} & 0_{6 \times dof} \\\\
0_{dof \times 6} & P
\end{bmatrix}
\nu^{idyn}
= J^{idyn} \nu^{idyn}
$$

So we have that:

$$
J^{idyn} = J^{pin} T =
J^{pin} \begin{bmatrix}
{}^B X_{velReprFrame} & 0_{6 \times dof} \\\\
0_{dof \times 6} & P
\end{bmatrix}
$$

Dividing the Jacobian in base and joint parts, this transformation can be expressed as:

$$
J^{idyn}\_{base} = J^{pin}\_{base} {}^B X_W
$$

$$
J^{idyn}\_{joint} = J^{pin}\_{joint} P
$$

### Model Acceleration

Both in iDynTree and pinocchio, the model velocity is just defined as the time derivative of the model acceleration, so we can just derivate the relation
that related to $\nu^{pin}$ and $\nu^{idyn}$ to obtain:

$$
\dot{\nu}^{pin} =
\dot{T} \nu^{idyn} + T \dot{\nu}^{idyn}
$$

where, as the permutation matrix $P$ is constant, i.e. $\dot{P} = 0$, so we have:

$$
\dot{T} =
\begin{bmatrix}
{}^B \dot{X}\_{velReprFrame} & 0\_{6 \times dof} \\\\
0\_{dof \times 6} & 0\_{dof \times dof}
\end{bmatrix}
$$

Where, thanks to equation 2.36 in https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf, ${}^B \dot{X}\_{velReprFrame} = {}^B {X}_{velReprFrame} {}^{velReprFrame} \mathrm{v}\_{B, velReprFrame} \times$.

As a recap, the following table provides the different quantities related  to $T$ and $\dot{T}$ for different values of $velReprFrame$:

| Velocity Representation Convention | velReprFrame |  ${}^B X_{velReprFrame}$ | ${}^{velReprFrame} \mathrm{v}\_{B, velReprFrame}$ |
|:-----------------------------------:|:--------------:|:---------------------:|:----------------------------------------------:|
| `BODY_FIXED_REPRESENTATION`         |  $B$          | ${}^B X_B = 1_{6 \times 6}$ |  |
| `INERTIAL_FIXED_REPRESENTATION`     |  $A$          | ${}^B X_A$                  |        |
| `MIXED_REPRESENTATION`              |  $B[A]$      |  ${}^B X_{B[A]} = $  |     |

### Frame Acceleration

Differently from frame velocities, for frame acceleration it is not possible to obtain the accelerations provided by the iDynTree interface (i.e. `KinDynComputations::getFrameAcc`) simply by specifing the `ReferenceFrame` parameter of the `pinocchio::getFrameAcceleration` function. This is because the semenatics of what "acceleration" is is different between iDynTree and pinocchio. In the case of iDynTree, the acceleration for a given representation is always the time derivative of the velocity in the same representation, while for pinocchio the time derivative is always done in the absolute/world frame, and then the resulting vector is projected back in the frame specified by `ReferenceFrame`. Similarly, `pinocchio` also provides a `pinocchio::getFrameClassicalAcceleration`, method, that does the derivative in the $F[A]$ frame, and project back
the resulting vector in the frame specified by `ReferenceFrame`.

In mathematical terms:

| Velocity Representation Convention (Pinocchio)  | `pinocchio::getFrameAcceleration` for frame $F$   | `pinocchio::getFrameClassicalAcceleration` for frame $F$  |
|:-----------------------------------:|:------------------------------------------------:|:------------------:|
| `LOCAL_WORLD_ALIGNED`               |  ${}^{F[A]} X_{A} {}^A \dot{v}_{A,F}$            | ${}^{F[A]} \dot{v}_{A,F}$            |
|   `WORLD`                           |  ${}^A \dot{v}_{A,F}$                            | ${}^{A} X_{F[A]} {}^{F[A]} \dot{v}_{A,F}$           |
|   `LOCAL`                           |  ${}^{F} X_{A} {}^A \dot{v}_{A,F}$               | ${}^{F} X_{F[A]} {}^{F[A]} \dot{v}_{A,F}$  |


| Velocity Representation Convention (iDynTree)  | `iDynTree::KinDynComputations::getFrameAcc` for frame $F$  |
|:-----------------------------------:|:------------------------------------------------:|
| `MIXED_REPRESENTATION`               |  ${}^{F[A]} \dot{v}_{A,F}$            |
|   `INERTIAL_FIXED_REPRESENTATION`                           |  ${}^A \dot{v}_{A,F}$  |
|   `BODY_FIXED_REPRESENTATION`                           |  ${}^{F} \dot{v}_{A,F}$               |

Given that ${}^{F} \dot{v}\_{A,F} = {}^F X\_A {}^A \dot{v}\_{A,F}$ (to demonstrated that, just use equation 2.36 in https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf), in a nutsheel this means that we can use `pinocchio::getFrameAcceleration` in the `INERTIAL_FIXED_REPRESENTATION` and `BODY_FIXED_REPRESENTATION`, and `pinocchio::getFrameClassicalAcceleration` for `MIXED_REPRESENTATION`.

### Dynamics

Both in the case of Pinocchio and iDynTree, the unconstrained dynamics is described similarly (for the sake of readability, we do not indicate in the following explicitly the dependencies on $q$ and $\nu$).

Pinocchio:

$$
M^{pin} \dot{\nu}^{pin} + C^{pin} + g^{pin} = \tau^{pin}
$$

iDynTree:

$$
M^{idyn} \dot{\nu}^{idyn} + C^{idyn} + g^{idyn} = \tau^{idyn}
$$

where in this context $\tau^{pin} \in \mathbb{R}^{n+dof}$ and $\tau^{pin} \in \mathbb{R}^{n+dof}$ represent the generalized torques, obtained by summing the internal torques and the effect of the external 6D force on each link.

Going from pinocchio to iDynTree is exactly a change of variables as defined in Section 3.6 of https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf, with the difference that in this case the transformation matrix $T$:

$$
\nu^{pin} = T \nu^{idyn}
= \begin{bmatrix}
{}^B X_{velReprFrame} & 0_{6 \times dof} \\\\
0_{dof \times 6} & P
\end{bmatrix}
\nu^{idyn}
$$

does not have an identity in the bottom right block, but rather a permutation matrix. However, the transformation formulas provided in that case also apply in this case, in particular using the Equation 3.60 of https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf we have:

$$
M^{pin} = T^{-T} M^{idyn} T^{-1}
$$
and
$$
\tau^{pin} = T^{-T} \tau^{idyn}
$$

from which:

$$
M^{idyn} = T^{T} M^{pin} T
$$
and
$$
\tau^{idyn} = T^{T} \tau^{idyn}
$$


Similar transformation apply to $C^{idyn}$ and $g^{idyn}$

#### Inverse Dynamics

Given the conversion provided in the previous section, we can define the Inverse Dynamics (ID) function for idyn as:

$$
ID^{idyn}(\dot{\nu}^{idyn}) = M^{idyn} \dot{\nu}^{idyn} + C^{idyn} + g^{idyn}
$$

and the ID of pinocchio:

$$
ID^{pin}(\dot{\nu}^{pin}) = M^{pin} \dot{\nu}^{pin} + C^{pin} + g^{pin}
$$

so we can implement $ID^{idyn}$ using $ID^{pin}$ as:

$$
ID^{idyn}(\dot{\nu}^{idyn}) = T^{T} ID^{pin}(\dot{\nu}^{pin}) = T^{T} ID^{pin}(T \dot{\nu}^{idyn} + \dot{T} \nu^{idyn})
$$

#### Inverse Dynamics  Bias Forces

A classical function provided by dynamics libraries is the one that provides just the coriolis and gravity forces, i.e.: 

$$
H^{idyn} = ID^{idyn}(0) = C^{idyn} + g^{idyn}
$$

By using the formula to express iDynTree's ID in term of Pinocchio's ID and substituting $\dot{\nu}^{idyn}$, it is possible to expressed iDynTree's bias forces in term of Pinocchio's ID:

$$
H^{idyn} = ID^{idyn}(0) = ID^{idyn}(0) = T^{T} ID^{pin}(\dot{T} \nu^{idyn})
$$

**Important Note: As highlighted in the previous formula, substituing $\dot{\nu}^{idyn} = 0$ is not equivalent as substituing $\dot{\nu}^{pin} = 0$.**

#### Forward Dynamics

Given the conversion provided in the previous section, we can define the Forward Dynamics (FD) function for idyn as:

$$
FD^{idyn}(\tau^{idyn}) = {(M^{idyn})}^{-1} \left( \tau^{idyn} - C^{idyn} - g^{idyn} \right)
$$

and the ID of pinocchio:

$$
FD^{pin}(\tau^{pin}) = {(M^{pin})}^{-1} \left( \tau^{pin} - C^{pin} - g^{pin} \right)
$$

so we can implement $FD^{idyn}$ using $FD^{pin}$ as:

$$
FD^{idyn}(\tau^{idyn}) = \text{totalAccPinToiDyn}(FD^{pin}(\tau^{pin})) = \text{totalAccPinToiDyn}(FD^{pin}(T^{-T} \tau^{idyn}))
$$

where:

$$
\text{totalAccPinToiDyn}(a) = T^{-1} (a - \dot{T} \nu^{idyn})
$$

