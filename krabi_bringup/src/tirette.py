#!/usr/bin/env python3
"""Tirette simulée.

Séquence d'une vraie tirette : insérée (False) puis retirée (True) = départ du match.

- auto_start = True  : comportement historique, le match démarre tout seul (~14 s après le lancement).
- auto_start = False : la tirette est insérée puis on attend le service `start_match`
                       (std_srvs/Trigger). Entre les deux, on peut lancer start_recalage /
                       go_to_start sur goal_strat.
"""
import threading
from time import sleep

import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('tirette_publisher')
    node.declare_parameter('auto_start', True)
    auto_start = node.get_parameter('auto_start').value
    publisher = node.create_publisher(Bool, 'tirette', 10)

    start_requested = threading.Event()

    def start_match_cb(_request, response):
        if start_requested.is_set():
            response.success = False
            response.message = 'match déjà lancé'
        else:
            start_requested.set()
            response.success = True
            response.message = 'tirette retirée : départ du match'
        node.get_logger().info(response.message)
        return response

    node.create_service(Trigger, 'start_match', start_match_cb)

    # Le service doit répondre pendant qu'on dort dans la boucle principale
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def publish_for(value, seconds):
        msg = Bool()
        msg.data = value
        for _ in range(int(seconds * 10)):
            publisher.publish(msg)
            sleep(0.1)

    sleep(9)  # Wait 9s, to account for the billig init.

    # Tirette insérée pendant 5 s (goal_strat : WAIT_TIRETTE_IN -> INIT -> WAIT_TIRETTE)
    publish_for(False, 5)

    if not auto_start:
        # On arrête de publier : goal_strat garde la dernière valeur (False)
        node.get_logger().info(
            'Match en attente. Lancer : ros2 service call /krabi_ns/start_match std_srvs/srv/Trigger')
        # Attente avec timeout pour pouvoir s'arrêter proprement (Ctrl+C / fin du launch)
        while not start_requested.wait(0.2):
            if not rclpy.ok():
                return

    msg = Bool()
    msg.data = True
    node.get_logger().info('Published: True (départ)')
    while rclpy.ok():
        publisher.publish(msg)
        sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
