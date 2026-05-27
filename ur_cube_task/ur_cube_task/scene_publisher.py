import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.msg import ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')

        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )

        # Vent litt så MoveIt er klar
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.published = False

    def make_box(self, obj_id, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = obj_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def make_color(self, obj_id, r, g, b, a=1.0):
        color = ObjectColor()
        color.id = obj_id
        color.color = ColorRGBA(r=r, g=g, b=b, a=a)
        return color

    def publish_scene(self):
        if self.published:
            return
        self.published = True

        scene = PlanningScene()
        scene.is_diff = True

        # Bord – juster z til faktisk bordhøyde
        table = self.make_box('table', 0.0, -0.2, -0.1, 0.8, 0.8, 0.02)
        scene.world.collision_objects.append(table)
        scene.object_colors.append(
            self.make_color('table', 0.6, 0.4, 0.2)
        )

        # Kuber – bruk koordinatene fra deteksjonen
        cubes = [
            ('red_cube',   -0.274, -0.175, 0.04,  1.0, 0.0, 0.0),
            ('green_cube', -0.402, -0.158, 0.04,  0.0, 0.8, 0.0),
            ('blue_cube',  -0.300,  0.005, 0.04,  0.0, 0.0, 1.0),
        ]

        for name, x, y, z, r, g, b in cubes:
            cube = self.make_box(name, x, y, z, 0.05, 0.05, 0.05)
            scene.world.collision_objects.append(cube)
            scene.object_colors.append(self.make_color(name, r, g, b))

        self.scene_pub.publish(scene)
        self.get_logger().info('Scene publisert – bord og kuber lagt til i RViz')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScenePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
