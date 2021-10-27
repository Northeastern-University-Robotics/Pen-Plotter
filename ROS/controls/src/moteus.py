#!/usr/bin/env python

from asyncio.tasks import sleep
import moteus
from moteus.moteus import Controller
import moteus_pi3hat
import asyncio
from threading import Thread
import math
import copy

class Moteus:

    def __init__(self, ids=[[1,2,3],[4,5,6],[7,8,9],[10,11,12]]) -> None:
        self.ids = ids
        self.exitFlag = False

        self.rawIds = []
        for bus in self.ids:
            for id in bus:
                self.rawIds.append(id)

        self.motor_states = {}
        for id in self.rawIds:
            self.motor_states[id] = {"position" : math.nan, "velocity" : 0, "torque" : 0}

        self.results = None

        self.moteus_thread = Thread(target=self.moteusMain, args=())
        self.moteus_thread.start()



    def setAttributes(self, canId:int, pos=None, velocity=None, torque=None):
        self.motor_states[canId]["position"] = pos
        self.motor_states[canId]["velocity"] = velocity
        self.motor_states[canId]["torque"] = torque

    def moteusMain(self):

        def createEventLoop():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            return asyncio.get_event_loop()

        loop = createEventLoop()
        asyncio.set_event_loop(loop)

        async def onClose(transport=None, servos = None):
            if(transport != None and servos != None):
                await transport.cycle([x.make_stop() for x in servos.values()])
            print("Done")
            pass

        async def onOpen(transport=None, servos = None):
            if(transport != None and servos != None):
                results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])

                results = self.getParsedResults(results)

                await transport.cycle([
                    x.make_position(
                    position = results[i]["POSITION"],
                    velocity = results[i]["VELOCITY"],
                    maximum_torque = results[i]["TORQUE"]
                    ) for i, x in enumerate(servos.values())
                ])
            print("Starting.....")
        
        async def main():

            servo_bus_map = {}
            for i in range(len(self.ids)):
                bus_ids = []
                for id in self.ids[i]:
                    bus_ids.append(id)
                servo_bus_map[i+1] = bus_ids

            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map = servo_bus_map
            )

            servos = {}
            for id in self.rawIds:
                servos[id] = moteus.Controller(id = id, transport=transport)


            await onOpen(transport, servos)

            raw_ids = self.rawIds


            while not self.exitFlag:

                commands = [
                    servos[id].make_position(
                        position = self.motor_states[id]["position"],
                        velocity = self.motor_states[id]["velocity"],
                        maximum_torque = self.motor_states[id]["torque"],
                        query = True) for id in raw_ids
                ]

                #print(self.results)
                self.results = await transport.cycle(commands)


            await onClose(transport, servos)

        asyncio.run(main())

    def closeMoteus(self):
        self.exitFlag = True
        self.moteus_thread.join()

    def getRawResults(self):
        return self.results

    def getParsedResults(self, results = None):
        parsed = []

        #print(self.results)

        if(results is None and self.results is not None):
            results = copy.deepcopy(self.results)
            print("HERE")
            if(results == None):
                print("Results are NONE!")
            #print(results)
        elif(results is None and self.results is None):
            return None

        try:
            for result in results:
                try:
                    parsed.append(
                        {
                            "MODE" : result.values[0x0],
                            "POSITION" : result.values[0x1],
                            "VELOCITY" : result.values[0x2],
                            "TORQUE" : result.values[0x3],
                            "VOLTAGE": result.values[0x00d],
                            "TEMPERATURE" : result.values[0x00e],
                            "FAULT" : result.values[0x00f]
                        }
                    )
                except:
                    print("-----HII-----")
                    print(results)
                    print("----------")
                    print(result)
                    print("----------")
                    traceback.print_exc()
                    print("------BIII----")
                    exit()
                    return None
                    
        except:
            print("----------")
            print(results)
            print("----------")
            print(self.results)
            print("ERROR")
            print("----------")
            exit()
        return parsed

import traceback
if __name__ == '__main__':
    m = Moteus(ids=[[1,2]])
    to = 0.03
    m.setAttributes(1, pos=100, velocity = 0.2, torque=to)
    torques = []
    while(True):
        value = m.getParsedResults()
        #print(value)
        if(value != None):
            torques.append(value[0]["TORQUE"])
            if(value[0]["POSITION"] > 100):
                m.setAttributes(1, pos=0, velocity =0.2, torque=to)
            elif(value[0]["POSITION"] < 0):
                m.setAttributes(1, pos=100, velocity =0.2, torque=to)
            print(max(torques))
        #m.getParsedResults()