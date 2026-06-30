# Instruction

Install pixi from https://pixi.prefix.dev/latest/installation/

```sh
curl -fsSL https://pixi.sh/install.sh | sh
```

# Clone repositiory
kria-som

```sh
git clone git@github.com:gbionics/ecp-playground.git --branch kria-som
```


# Build with pixi
```sh
pixi run build
cd build
```

# Test
```sh
./actuator-cli ../config/gene-000.toml 
```