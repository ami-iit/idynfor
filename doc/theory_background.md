# iDynFor theory background

This document provide an overview of the different semantics between iDynTree and Pinocchio
data-structures and interfaces, and provide a reference for the implementation in iDynFor classes, that implements iDynTree-inspired interfaces using Pinocchio.

## iDynTree::Model vs pinocchio::Model

### Link and Joints

In `iDynTree::Model` a multibody is composed by its links that are interconnected by joints.
Each link is connected to another link by a joint, so you can't attach a joint to another joint
without having a link in the middle. Furthermore, `iDynTree::Model` consider all models to be
floating base by default, so there is no "joint" connecting the base to "universe" or "world" frame. Only links have a frame w.r.t. to which you can compute forward kinematics or the jacobian, while there is no explicit concept of "joint frame"

On the other hand, in `pinocchio::Model` joints are interconnected to each other, to build
the so-called "kinematic tree". So multiple joints can be connected to each other, without having a body in the middle. On the other hand, each link must have a parent joint, even if it is the first body of the kinematic tree.
A model can be connected to the "universe" reference frame with any kind of frame, but for consistency with `iDynTree` the pinocchio models built in `iDynFor` are always connected to "universe" with a `pinocchio::JointFreeFlyer` (i.e. 6-DOF joint).

For these reasons, the count of links and joints in `iDynTree::Model` and  `pinocchio::Model` are differents.
For links, in iDynTree only the internal bodies that compose the multibody model are counted, while for `pinocchio` also the `universe` "link" is considered.
For joints, in iDynTree only the joints that internconnect internal links are considered, while in `pinocchio` two additional joints are considerd:
* the joint that connects the floating base to `universe`,
* the  parent joint of `universe`.

We will always have that for corresponding models:
* `iDynTree::Model::getNrOfLinks() + 1 == pinocchio::Model::nbodies` and
* `iDynTree::Model::getNrOfJoints() + 2 == pinocchio::Model::njoints`.

As an example, if you have a single link floating base model, this is the number of its joints and links w.r.t. if modeled used `iDynTree::Model` and  `pinocchio::Model`:

| Quantity |  `iDynTree::Model`  | `pinocchio::Model` |
|:--------:|:---------------------:|:--------------------:|
| Number of Links | 1             |          2            |
| Number of Joints | 0             |         2            |

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




