# -> Bibliotecas:


import os  # monta caminhos de arquivo (join, dirname) e lê variáveis de ambiente

from ament_index_python.packages import get_package_share_directory  # acha a pasta 'share' de um pacote ROS instalado

from launch import LaunchDescription  # container principal que agrupa tudo que o launch vai executar

from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
# IncludeLaunchDescription -> inclui outro arquivo de launch (usado pro launch do Gazebo)
# RegisterEventHandler     -> dispara uma ação quando outro processo termina (usado pra encadear os spawners)
# SetEnvironmentVariable   -> define variável de ambiente pro processo (usado no GAZEBO_MODEL_PATH)
# TimerAction              -> atrasa a execução de uma ação por N segundos

from launch.event_handlers import OnProcessExit  # evento: dispara quando um processo é encerrado

from launch.launch_description_sources import PythonLaunchDescriptionSource  # permite incluir um launch.py de outro pacote

from launch_ros.actions import Node  # sobe um node ROS2 (executável) como parte do launch

import xacro  # converte o .xacro (com macros/parâmetros) em URDF puro

 
# -> Código principal:


def generate_launch_description():
    pkg_description = get_package_share_directory('fin_ray_description')

    # Configura o GAZEBO_MODEL_PATH: Para o Gazebo Classic achar as meshes/models do pacote e o robô/objeto 
    pkg_parent_dir = os.path.dirname(pkg_description)
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{pkg_parent_dir}:{os.path.expanduser('~/.gazebo/models')}:{current_model_path}"

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # Expande o .xacro (macros, parâmetros) em um URDF puro que o ROS entende
    xacro_file = os.path.join(pkg_description, 'urdf', 'gripper.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Publica o TF de cada link a partir do URDF, usado pelo RViz/Gazebo
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    world_file = os.path.join(pkg_description, 'worlds', 'fin_ray.world')

    # Sobe o Gazebo Classic já carregando o world file definido acima
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Instancia o robô no Gazebo a partir do robot_description publicado acima
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fin_ray_gripper'],
        output='screen'
    )

    # Instancia o objeto de teste (cubo/cilindro) já no início
    spawn_objeto_teste = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'objeto_teste',
            '-file', os.path.join(pkg_description, 'objects', 'objeto_teste.sdf'),
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Ativa o broadcaster que publica o estado (posição/velocidade) das juntas em /joint_states
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Ativa o controller que comanda as 4 juntas revolute da garra
    load_fin_ray_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fin_ray_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Espera 5s antes de subir o broadcaster, dar tempo do controller_manager existir
    delay_broadcaster = TimerAction(
        period=5.0,
        actions=[load_joint_state_broadcaster]
    )

    # Só chama o controller da garra quando o broadcaster terminar de subir (evita race condition)
    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_fin_ray_controller],
        )
    )

    return LaunchDescription([
        set_gazebo_model_path,              # precisa estar setado antes do Gazebo subir
        node_robot_state_publisher,         # publica o TF a partir do robot_description
        gazebo,                             # sobe o Gazebo Classic com o world file
        spawn_entity,                       # instancia o robô dentro da simulação
        spawn_objeto_teste,                 # instancia o objeto de teste dentro da simulação
        delay_broadcaster,                  # sobe o joint_state_broadcaster após 5s
        delay_controller_after_broadcaster, # sobe o fin_ray_controller quando o broadcaster terminar
    ])
