# Fin Ray Gripper

Garra robótica baseada no efeito Fin Ray, desenvolvida em ROS2 Humble + Gazebo Classic pelo Laboratório de Engenharia de Sistemas e Robótica, como parte de um projeto de Iniciação Científica.

> **Status:** versão preliminar (v0.1). Instabilidades podem ocorrer -> otimizações e ajuste fino de parâmetros seguem em desenvolvimento.

## O que tem neste repositório

- **`fin_ray_description/`** -> pacote ROS2 com a descrição da garra (URDF/Xacro), meshes, launch files, configuração dos controllers e o nó de movimentação.
- **`FINRAY_GRIPPER_MANUAL.pdf`** -> documento informativo detalhando o projeto: por que o efeito Fin Ray foi escolhido, como a deformação foi implementada no Gazebo Classic (modelo de corpo pseudo-rígido), como o agarre é garantido (IFRA Link Attacher), estrutura de pastas, tutorial de execução e como o URDF e o nó `gripper_mover.cpp` estão organizados.

Para entender a fundo as decisões de design e a lógica do código, consulte o PDF, este README cobre só o essencial pra rodar o projeto.

## Pré-requisitos

- Ubuntu 22.04 + ROS2 Humble
- Gazebo Classic
- Pacotes ROS2: `ros2_control`, `ros2_controllers`, `gazebo_ros2_control`, `gazebo_ros_pkgs`, `xacro`
- [IFRA Link Attacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher) -> precisa estar clonado dentro do `src/` do workspace, junto com o `fin_ray_description` (fornece o plugin `libgazebo_link_attacher.so` e as mensagens `linkattacher_msgs`)

## Build

```bash
cd ~/finray_ws
colcon build
source install/setup.bash
```

## Como executar

Você vai precisar de 3 janelas de terminal.

**Janela 1 -> Sobe o Gazebo com a garra**
```bash
ros2 launch fin_ray_description gazebo.launch.py
```

**Janela 2 -> Nó de movimentação**
```bash
ros2 run fin_ray_description gripper_mover
```

**Janela 3 -> Envio de comandos**
```bash
ros2 topic pub --once /comando_garra std_msgs/msg/String "data: 'fechar'"
```
```bash
ros2 topic pub --once /comando_garra std_msgs/msg/String "data: 'abrir'"
```

## Estrutura do pacote

| Pasta/Arquivo | Utilidade |
|---|---|
| `config/` | Configuração dos controllers (`controllers.yaml`) |
| `launch/` | Launch file que sobe Gazebo, robot state publisher, spawners e controllers |
| `meshes/` | Malhas 3D (.stl) de cada peça da garra |
| `objects/` | Modelo (.sdf) do objeto de teste |
| `src/` | Nó `gripper_mover.cpp`, controla abertura/fechamento e o agarre via Link Attacher |
| `urdf/` | Descrição da garra em Xacro (`gripper.urdf.xacro`) |
| `worlds/` | Mundo do Gazebo (`fin_ray.world`) |

Mais detalhes sobre cada um desses pontos, incluindo a lógica de detecção de contato e o diagrama de estados do nó de movimentação, estão no documento em PDF.

## Autor

Ronaldo Urquiza Herculano Filho — LASER, UFPB
