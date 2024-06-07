# TODO: write the patrol FSM action
from typing import List

from .pddl import room_type, room_at, room_patrolled
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fms_action import Merlin2BasicStates
from merlin2_fms_action import Merlin2FmsAction

from kant_dto import PddlObjectDto, PddlConditionEffectDto

from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin import CbState

import rclpy

from geometry_msgs.msg import Twist

class Merlin2RoomPatrolFmsAction(Merlin2FmsAction):

    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp=PddlObjectDto(wp_type, "wp")

        super().__init__("room_patrol")

        self.rotationPub = self.create_subscription(
            Twist,
            'cmd_vel',
            10)

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED],self.prepare_text),
            transitions={
                SUCCEED: "SPEAKING"
            }
        )

        self.add_state(
            "SPEAKING",
            tts_state
        )

    def rotate(self, blackboard : Blackboard) -> str:
        msg = Twist()
        rate = self.create_rate(5)
        msg.linear.x = 0.0

        msg.angular.z = 0.5
        self.rotationPub.publish(msg)

        rate.sleep()

        msg.angular.z = 0.0
        self.rotationPub(msg)

        return SUCCEED

    def prepare_text(self, blackboard : Blackboard) -> str:

#        room_name = blackboard.merlin2_action_goal.objects[0][-1]
        blackboard.text = f"Strecher room patrolled"

        return SUCCEED

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]
    
    def create_conditions(self) -> List[PddlConditionEffectDto]:

        cond_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            PddlConditionEffectDto.AT_START,
            is_negative=False
        )

        cond_2 = PddlConditionEffectDto(
            robot_at,
            [self._wp],
            PddlConditionEffectDto.AT_START
        )

        cond_3 = PddlConditionEffectDto(
            room_at,
            [self._room, self._wp],
            PddlConditionEffectDto.AT_START
        )

        return [cond_1, cond_2, cond_3]
    
    def create_effects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]

def main():
    rclpy.init()
    node = Merlin2RoomPatrolFmsAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "main":
    main()