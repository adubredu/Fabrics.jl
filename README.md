# Fabrics

Julia implementation of [Geometric Fabrics for the Acceleration-based Design of Robotic Motion](https://arxiv.org/abs/2010.14750).

## Installation
1. Open your Julia REPL by typing  `julia` in your terminal.
2. Press `]` on your keyboard to enter the package manager
3. Enter command `add https://github.com/adubredu/Fabrics.jl` and press 
`Enter` on your keyboard to install this package.
4. Press the `Backspace` key on your keyboard to return to the REPL

## Usage
I implement Geometric Fabrics for 3 Planar robot systems viz; A point mass, A planar robot arm and PickleRick, a planar humanoid robot.

To run the Point mass Navigator example, go back to your Julia REPL, activate the environment where you installed this packagage and run the following commands:

```
using Fabrics
path = pathof(Fabrics)
include(joinpath(path, "examples/pointmass_eg.jl))
```

To run the Planar Arm example, go back to your Julia REPL, activate the environment where you installed this packagage and run the following commands:

```
using Fabrics
path = pathof(Fabrics)
include(joinpath(path, "examples/planararm_eg.jl))
```

To run the PickleRick example, go back to your Julia REPL, activate the environment where you installed this packagage and run the following commands:

```
using Fabrics
path = pathof(Fabrics)
include(joinpath(path, "examples/picklerick_eg.jl))
```