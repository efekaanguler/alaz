from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Sizin 2D Lidar dönüştürücüsünden gelen veriyi (pointcloud_raw),
        # Autoware'in beklediği ana topik ismine (concatenated/pointcloud) yönlendiriyoruz.
        # Bu işlem CPU yormaz, sadece veri akışını sağlar.
        
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_lidar_pointcloud',
            output='screen',
            arguments=[
                '/sensing/lidar/top/pointcloud_raw',       # GİRDİ: Lidar.launch.xml'den gelen veri
                '/sensing/lidar/concatenated/pointcloud'   # ÇIKTI: Autoware'in beklediği ana veri
            ],
        ),
    ])
