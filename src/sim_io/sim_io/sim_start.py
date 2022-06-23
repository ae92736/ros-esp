#!/usr/bin/python3
# -*- coding: utf-8 -*-

from environs import Env
from rclpy.node import Node
import rclpy, lgsvl, time

class SimulationStarter(Node):
    def __init__(self):
        super().__init__('SimulationStarter')
        self.map_id = "6c6e42c1-ef58-4f7f-8a58-19578f99c174"
        self.vehicle_id = "c44cfd8b-f3f8-4bbc-a248-31ffbfe47898"
        self.env = Env()

        self.sim = lgsvl.Simulator("localhost", 8181)
        self.sim.load(self.map_id)

        self.egoState = lgsvl.AgentState()
        self.egoState.transform.position = lgsvl.Vector(15.71, 0, 1.5)
        self.egoState.transform.rotation.y = 270
        self.ego = self.sim.add_agent(self.vehicle_id, lgsvl.AgentType.EGO, self.egoState)

        self.triple_control = ["trigger=250;red=11.5;yellow=0.5;green=6;loop",
                               "trigger=250;green=6;red=11.5;yellow=0.5;loop",
                               "trigger=250;red=5.5;yellow=0.5;green=6;red=6;loop"]
        self.quadruple_control = ["trigger=250;red=17.5;yellow=0.5;green=6;loop",
                                  "trigger=250;green=6;red=17.5;yellow=0.5;loop",
                                  "trigger=250;red=5.5;yellow=0.5;green=6;red=12;loop",
                                  "trigger=250;red=11.5;yellow=0.5;green=6;red=6;loop"]

        self.get_logger().info("Simulation started")

    def get_lights(self):
        self.intersection1 = list()
        self.intersection1.append(self.sim.get_controllable(lgsvl.Vector(-7.80000019073486, 2.73300004005432, -34.0283432006836), "signal"))
        self.intersection1.append(self.sim.get_controllable(lgsvl.Vector(-16.9778633117676, 2.73300004005432, -34.1227951049805), "signal"))
        self.intersection1.append(self.sim.get_controllable(lgsvl.Vector(-16.8358631134033, 2.73300004005432, -25.156795501709), "signal"))
        
        self.intersection2 = list()
        self.intersection2.append(self.sim.get_controllable(lgsvl.Vector(-67.117431640625, 2.73300004005432, -4.98242473602295), "signal"))
        self.intersection2.append(self.sim.get_controllable(lgsvl.Vector(-67.2804336547852, 2.73300004005432, 4.45657539367676), "signal"))
        self.intersection2.append(self.sim.get_controllable(lgsvl.Vector(-76.2184295654297, 2.73300004005432, -4.18942451477051), "signal"))
        
        self.intersection3 = list()
        self.intersection3.append(self.sim.get_controllable(lgsvl.Vector(-76.4820022583008, 2.73300004005432, -25.1622772216797), "signal"))
        self.intersection3.append(self.sim.get_controllable(lgsvl.Vector(-67.4820022583008, 2.73300004005432, -34.3312759399414), "signal"))
        self.intersection3.append(self.sim.get_controllable(lgsvl.Vector(-76.7180023193359, 2.73300004005432, -34.1462783813477), "signal"))
        self.intersection3.append(self.sim.get_controllable(lgsvl.Vector(-67.6439971923828, 2.73300004005432, -25.4069366455078), "signal"))

    def program_lights(self):
        self.get_lights()
        for i in range(0,3):
            self.intersection1[i].control(self.triple_control[i])
            self.intersection2[i].control(self.triple_control[i])
            self.intersection3[i].control(self.quadruple_control[i])

        self.intersection3[3].control(self.quadruple_control[3])

    def connect_bridge(self):
        self.ego.connect_bridge(self.env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), self.env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))
        while not self.ego.bridge_connected:
            self.get_logger().info("waiting for bridge to connect")
            time.sleep(0.5)
        
        self.get_logger().info("Bridge got connected")

def main(args=None):
    rclpy.init(args=args)
    simulation = SimulationStarter()
    simulation.program_lights()
    simulation.connect_bridge()
    
    try:
        simulation.sim.run()

    except KeyboardInterrupt:
        simulation.get_logger().info("Simulation is closing")
        simulation.sim.stop()

    rclpy.shutdown()

if __name__ == "__main__":
    main(args)