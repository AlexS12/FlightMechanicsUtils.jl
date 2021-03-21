# FlightMechanicsUtils

[![Build Status](https://github.com/AlexS12/FlightMechanicsUtils.jl/workflows/CI/badge.svg)](https://github.com/AlexS12/FlightMechanicsUtils.jl/actions)
[![Documentation](https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat-square)](https://alexs12.github.io/FlightMechanicsUtils.jl/dev)
[![License](https://img.shields.io/badge/license-MIT-blue.svg?style=flat-square)](https://github.com/AlexS12/FlightMechanicsUtils.jl/blob/main/LICENSE)


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


## Contributing

If you are using or want to use this package and have any suggestion or found a bug, open an [issue](https://github.com/AlexS12/FlightMechanicsUtils.jl/issues).
