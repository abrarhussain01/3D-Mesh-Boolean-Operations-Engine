# 3D Mesh Boolean Operations Engine

A lightweight C++ engine that performs **boolean operations** (Union, Intersection, Difference) on arbitrary 3D triangle meshes — the same class of algorithms powering CAD tools like Autodesk Fusion 360 and AutoCAD.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Algorithm](#algorithm)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Example Output](#example-output)
- [Mathematical Background](#mathematical-background)
- [Complexity Analysis](#complexity-analysis)
- [Limitations & Future Work](#limitations--future-work)
- [References](#references)

---

## Overview

Boolean operations on 3D meshes answer the question:

> *Given two solid objects A and B represented as triangle meshes, compute the mesh that represents A ∪ B, A ∩ B, or A − B.*

This is a fundamental problem in **computational geometry** and is used extensively in:
- CAD/CAM modeling (Autodesk, SolidWorks)
- Game engine collision geometry
- 3D printing support structure generation
- Medical imaging (organ segmentation)
- Finite Element Method (FEM) mesh generation

---

## Features

- ✅ Three boolean operations: **Union**, **Intersection**, **Difference**
- ✅ Möller–Trumbore ray–triangle intersection (numerically robust)
- ✅ Ray casting point-in-mesh test using parity counting
- ✅ AABB (Axis-Aligned Bounding Box) spatial indexing for pruning
- ✅ Winding-order correction on subtracted faces (consistent normals)
- ✅ Works on any closed, watertight triangle mesh
- ✅ Zero external dependencies — pure C++17

---

## Algorithm

The engine uses a **centroid classification** approach:

```
For each triangle T in mesh A:
    Compute centroid C of T
    Cast a ray from C in +X direction
    Count intersections with mesh B
    If count is odd  → C is INSIDE B
    If count is even → C is OUTSIDE B

    Include T in result based on operation:
        UNION        → keep if OUTSIDE B
        INTERSECTION → keep if INSIDE  B
        DIFFERENCE   → keep if OUTSIDE B

Repeat symmetrically for triangles in mesh B.
For DIFFERENCE, flip winding order of included B triangles.
```

### Ray–Triangle Intersection: Möller–Trumbore

Given ray origin **O**, direction **D**, and triangle vertices **V₀, V₁, V₂**:

```
E1 = V1 - V0
E2 = V2 - V0
h  = D × E2
a  = E1 · h

if |a| < ε  →  ray is parallel, no intersection

f = 1 / a
s = O - V0
u = f (s · h)         if u ∉ [0,1] → miss

q = s × E1
v = f (D · q)         if v < 0 or u+v > 1 → miss

t = f (E2 · q)        if t > ε → intersection at O + tD
```

---

## Getting Started

### Prerequisites

- C++17 compatible compiler (g++ 9+, clang++ 10+, MSVC 2019+)
- CMake 3.14+ *(optional, for build system)*

### Compile & Run

**Direct:**
```bash
g++ -O2 -std=c++17 main.cpp -o mesh_bool
./mesh_bool
```

**With CMake:**
```bash
mkdir build && cd build
cmake ..
make
./mesh_bool
```

---

## Usage

```cpp
#include "mesh_boolean.h"

// Create two overlapping cubes
auto cubeA = makeCube({0, 0, 0}, 2.0);
auto cubeB = makeCube({1, 1, 1}, 2.0);

// Perform operations
auto uni   = booleanOp(cubeA, cubeB, UNION);
auto inter = booleanOp(cubeA, cubeB, INTERSECTION);
auto diff  = booleanOp(cubeA, cubeB, DIFFERENCE);

std::cout << "Union triangles:        " << uni.size()   << "\n";
std::cout << "Intersection triangles: " << inter.size() << "\n";
std::cout << "Difference triangles:   " << diff.size()  << "\n";
```

### Supported Operations

| Enum           | Meaning              | Description                          |
|----------------|----------------------|--------------------------------------|
| `UNION`        | A ∪ B                | All geometry from both meshes        |
| `INTERSECTION` | A ∩ B                | Only the overlapping region          |
| `DIFFERENCE`   | A − B                | A with B subtracted from it          |

---

## Example Output

Running the demo with two overlapping unit cubes of size 2.0, offset by (1,1,1):

```
Union triangles:        20
Intersection triangles: 4
Difference triangles:   16
```

Visual interpretation:

```
   Cube A          Cube B         Union (A ∪ B)
  ┌──────┐       ┌──────┐        ┌────────┐
  │      │       │  ┌───┼──┐     │        ├──┐
  │      ├───────┼──┤   │  │  =  │        │  │
  │      │       │  │   │  │     │        │  │
  └──────┘       └──┼───┘  │     └────────┘  │
                    └──────┘              └──┘
```

---

## Mathematical Background

### Point-in-Mesh Test (Ray Casting)

Based on the **Jordan curve theorem** extended to 3D: a ray from any point crosses the boundary of a closed solid an **even** number of times if the point is outside, and an **odd** number of times if inside.

### Triangle Normal & Winding Order

For a triangle with vertices V₀, V₁, V₂ (counter-clockwise when viewed from outside):

```
Normal N = (V₁ − V₀) × (V₂ − V₀),  normalized
```

For DIFFERENCE, triangles from B that end up in the result must have their normals **flipped** (swap V₁ ↔ V₂) to point inward — correctly representing the carved surface.

### AABB Intersection Test

Two AABBs overlap iff they overlap on **all three axes** simultaneously:

```
overlap = (A.min.x ≤ B.max.x AND A.max.x ≥ B.min.x)
        AND (A.min.y ≤ B.max.y AND A.max.y ≥ B.min.y)
        AND (A.min.z ≤ B.max.z AND A.max.z ≥ B.min.z)
```

---

## Complexity Analysis

| Step                    | Time Complexity  | Notes                              |
|-------------------------|------------------|------------------------------------|
| AABB construction       | O(n)             | Per mesh, one pass over triangles  |
| Point-in-mesh (naive)   | O(n · m)         | n query points, m mesh triangles   |
| AABB-pruned PIM test    | O(n log m)       | With spatial indexing              |
| Full boolean operation  | O((n+m) log(nm)) | Both meshes processed              |

> **n** = triangles in mesh A, **m** = triangles in mesh B

---

## Limitations & Future Work

**Current Limitations:**
- Assumes **closed, watertight** meshes (no holes or self-intersections)
- Does not handle **coplanar triangles** between the two meshes
- No actual **mesh re-triangulation** at intersection boundaries (approximation via centroid classification)

**Planned Improvements:**
- [ ] BSP-tree or BVH acceleration for large meshes
- [ ] Exact intersection curve computation at mesh boundaries
- [ ] Re-triangulation of intersection seam using ear clipping
- [ ] OBJ file import/export for real mesh data
- [ ] OpenMP parallelization for per-triangle classification

---

*Built as part of a computational geometry portfolio targeting CAD/geometry software engineering roles.*
