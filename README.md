# Elkapod nav2 stack
![ROS2 distro](https://img.shields.io/badge/ros--version-jazzy-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.12-g.svg)

## Usage
Pull prebuild containers from the github container registry

```bash
docker pull ghcr.io/hexapodbionik/elkapod-navigation
```

### Deployment

```bash
docker compose --profile run up -d
docker exec -it elkapod-navigation bash
```

### Development

```bash
docker compose --profile dev up -d
docker exec -it elkapod-navigation-dev bash
```

