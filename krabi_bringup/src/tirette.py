#!/usr/bin/env python3
"""Tirette simulée.

Comme sur le vrai robot, le robot démarre sans tirette : le topic `tirette` vaut True (absente).
Le match ne part que sur la séquence complète : insérée (False), puis retirée (True).

- auto_start = True  : comportement historique, la tirette est insérée ~9 s après le lancement,
                       puis retirée 5 s plus tard : le match démarre tout seul.
- auto_start = False : on pilote la tirette avec deux services (std_srvs/Trigger) :
                         insert_tirette : insère la tirette (le robot s'initialise)
                         start_match    : la retire (départ du match)
                       Avant, pendant ou après l'insertion, on peut lancer un recalage.
"""
import threading

import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

ABSENT, INSERTED, REMOVED = 'absente', 'insérée', 'retirée'


def main():
    rclpy.init()
    node = rclpy.create_node('tirette_publisher')
    node.declare_parameter('auto_start', True)
    auto_start = node.get_parameter('auto_start').value
    publisher = node.create_publisher(Bool, 'tirette', 10)

    lock = threading.Lock()
    state = {'value': ABSENT}

    def move(expected, new, ok_message, refusal):
        def callback(_request, response):
            with lock:
                response.success = state['value'] == expected
                if response.success:
                    state['value'] = new
            response.message = ok_message if response.success else refusal
            node.get_logger().info(response.message)
            return response
        return callback

    node.create_service(Trigger, 'insert_tirette', move(
        ABSENT, INSERTED, 'tirette insérée',
        'refusé : tirette déjà insérée, ou match lancé'))
    node.create_service(Trigger, 'start_match', move(
        INSERTED, REMOVED, 'tirette retirée : départ du match',
        "refusé : insérer d'abord la tirette (insert_tirette), ou match déjà lancé"))

    # Publiée en continu, comme le GPIO du vrai robot : True = absente / retirée
    def publish():
        with lock:
            inserted = state['value'] == INSERTED
        publisher.publish(Bool(data=not inserted))

    node.create_timer(0.1, publish)

    if auto_start:
        # Sequence on a 1 s timer: inserted at 9 s (time for the billig init), removed 5 s later
        elapsed = {'s': 0}

        def sequence():
            elapsed['s'] += 1
            with lock:
                if elapsed['s'] == 9 and state['value'] == ABSENT:
                    state['value'] = INSERTED
                    node.get_logger().info('tirette insérée (auto_start)')
                elif elapsed['s'] == 14 and state['value'] == INSERTED:
                    state['value'] = REMOVED
                    node.get_logger().info('Published: True (départ)')

        node.create_timer(1.0, sequence)
    else:
        node.get_logger().info(
            'Tirette absente. Insérer : ros2 service call /krabi_ns/insert_tirette '
            'std_srvs/srv/Trigger ; départ : ros2 service call /krabi_ns/start_match '
            'std_srvs/srv/Trigger')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
