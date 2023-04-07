# iDynFor

**iDynFor** is a C++ library thas provides a compatibility layer between [`iDynTree`](https://github.com/robotology/idyntree) and [`pinocchio`](https://github.com/stack-of-tasks/pinocchio) multi-body dynamics libraries. In a nutshell, `iDynFor` provides you with classes that expose
the same methods of [`iDynTree::KinDynComputations`](https://robotology.github.io/idyntree/classiDynTree_1_1KinDynComputations.html), but using `pinocchio` for the actual implementation.

The library implemented in this repository is still experimental and API and ABI can change at every patch release.

## Theory

For all the theorical details on the relation between the formalism used in iDynTree/iDynFor and the one used in pinocchio, check the [iDynFor theory documentation](./doc/theory_background.md).

## Dependencies

The C++ dependencies required by idynfor are:
* [`iDynTree`](https://github.com/robotology/idyntree)
* [`pinocchio`](https://github.com/stack-of-tasks/pinocchio)

### Install dependencies with conda-forge

~~~
mamba create -n idynfordev cmake compilers make ninja pkg-config idyntree pinocchio catch2 eigenpy
~~~

Then, execute all the other commands after activating the environment:
~~~
mamba activate idynfordev
~~~

## ⚒️ Build the library

You can build the library coping and paste the following snippet into a terminal
```console
git clone https://github.com/ami-iit/idynfor
cd idynfor
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<install_prefix> ..
cmake --build .
[sudo] cmake --install .
```

## 🐛 Bug reports and support

All types of [issues](https://github.com/ami-iit/idynfor/issues/new) are welcome.

## 📝 License

Materials in this repository are distributed under the following license:

> All software is licensed under the BSD 3-Clause "New" or "Revised" License. See [LICENSE](https://github.com/GiulioRomualdi/meshcat-cpp/blob/master/LICENSE) file for details.
