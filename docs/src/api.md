# API

## Constants

### Air

```@docs
γ_AIR
R_AIR
```

### Earth

```@docs
gD
```


## Ellipsoid

Available ellipsoid models:

| Name                   |     a (m)     |     ``f^{-1}``   |
| :--------------------: | :------------ | :--------------- |
| `Clarke1866`           |  6378206.4    |    294.9786982   |
| `Clarke1880`           |  6378249.145  |    294.465       |
| `International`        |  6378388.0    |    297.0         |
| `Bessel`               |  6377397.155  |    299.1528128   |
| `Everest`              |  6377276.345  |    300.8017      |
| `ModifiedEverest`      |  6377304.063  |    300.8017      |
| `AustralianNational`   |  6378160.0    |    298.25        |
| `SouthAmerican1969`    |  6378160.0    |    298.25        |
| `Airy`                 |  6377564.396  |    299.3249646   |
| `ModifiedAiry`         |  6377340.189  |    299.3249646   |
| `Hough`                |  6378270.0    |    297.0         |
| `Fischer1960SouthAsia` |  6378155.0    |    298.3         |
| `Fischer1960Mercury`   |  6378166.0    |    298.3         |
| `Fischer1968`          |  6378150.0    |    298.3         |
| `WGS60`                |  6378165.0    |    298.3         |
| `WGS66`                |  6378145.0    |    298.25        |
| `WGS72`                |  6378135.0    |    298.26        |
| `WGS84`                |  6378137.0    |    298.257223563 |


```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["ellipsoid.jl"]
Private = false
```

## Rotations

### Coordinate systems

- Earth
- Local horizon
- Body
- Wind

```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["rotations.jl"]
Private = false
```


## Atmosphere

```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["atmosphere.jl"]
Private = false
```


## Kinematics

```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["kinematics.jl"]
Private = false
```

## Mechanics

```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["mechanics.jl"]
Private = false
```


## Anemometry

- Calibrated
- Equivalent
- True

```@autodocs
Modules = [FlightMechanicsUtils]
Pages   = ["anemometry.jl"]
Private = false
```
