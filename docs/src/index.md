# FlightMechanicsUtils.jl

*Flight Mechanics in Julia.*

## Overview

This is a suite for Flight Mechanics written in Julia. The purpose of this package is to supply efficient and validated Julia implementations of common Flight Mechanics calculations.

At the moment, it covers:

- International Standard Atmosphere.
- Transformations between common coordinate systems in Flight Mechanics problems (body, horizon, wind) supporting Euler angles and quaternions.
- Anemometric functions.

## Install

Last release:

`pkg> add FlightMechanicsUtils`

Dev version:

`pkg> dev FlightMechanicsUtils`

Run tests:

`pkg> test FlightMechanicsUtils`

## What's new

### v0.1.2-dev

#### New

- [`Ellipsoid`](@ref) type with some common ellipsoids such as `WGS84`.
- Transformation ([`ecef2llh`](@ref)) from ECEF (Earth Centered Earth Fixed) to LLH (latitude, longitude, height) and viceversa ([`llh2ecef`](@ref)) given the reference ellipsoid.
- Rotation ([`horizon2ecef`](@ref)) from local horizon to ECEF axis and vicecersa ([`ecef2horizon`](@ref)).


#### Enhancements

- [`coordinated_turn_bank(ψ_dot, α, β, tas, γ, g)`](@ref) now accepts gravity as optional argument.

### v.0.1.1

Initial [release](https://github.com/AlexS12/FlightMechanicsUtils.jl/releases/tag/v0.1.1).

## Contents

```@contents
Pages = ["index.md", "api.md"]
```
