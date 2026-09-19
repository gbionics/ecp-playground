# ecp-playground

## Build

```bash
pixi shell
pixi run build
```

## Usage

### GUI

```bash
pixi run gui-capabilities
./build/gui/actuator-test-gui [config.toml]
```

### Console tool

```bash
pixi run capabilities
./build/actuator-test-spline [config.toml]
```

If capabilities are not applied ahead of time, run either executable with
`sudo` instead. When no config path is passed, both tools default to
`../config/gene-000.toml`.
