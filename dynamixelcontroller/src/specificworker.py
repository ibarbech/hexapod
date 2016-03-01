#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time,collections

from PySide import *

from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"JointMotor.ice")
from RoboCompJointMotor import *

from jointmotorI import *

#from threading import Lock
from mutex	import *

from pydynamixel import dynamixel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.mutmState=mutex()
		self.busParams=BusParams()
		self.motores={}
		self.SDK=False
		self.lisPos=collections.deque()
		self.lisVel=collections.deque()
		self.mstateMap ={}

		self.setParams()
#		try:
		self.ser = dynamixel.get_serial_for_url(self.busParams.device,self.busParams.baudRate)
		for m in self.motores:
			dynamixel.init(self.ser,m.busId)
#		except Exception as e:
#			print('Unable to move to desired position.')
 #       	print e
		self.timer.timeout.connect(self.compute)
		self.Period = 100
		self.timer.start(self.Period)

	def setParams(self):
		try:
			name="Dynamixel"
			self.busParams.numMotors=int(self.configGetString(name+".NumMotors"))
			self.busParams.device=self.configGetString(name+".Device")
			self.busParams.baudRate=int(self.configGetString(name+".BaudRate"))
			self.busParams.basicPeriod=int(self.configGetString(name+".BasicPeriod"))
			self.SDK=self.configGetString(name+".SDK")in("true")
			if self.busParams.numMotors!="default":
				for i in range(self.busParams.numMotors):
					params=self.configGetString(name+".Params_"+str(i))
					aux=params.split(",")
					if "false" in aux[2]:
						invert=False
					elif "true" in aux[2]:
						invert=True
					m=MotorParams()
					m.invertedSign=invert
					m.busId=int(aux[1])
					m.minPos=float(aux[3])
					m.maxPos=float(aux[4])
					m.maxVelocity=int(aux[6])
					m.zeroPos=int(aux[5])
					m.stepsRange=int(aux[7])
					m.maxDegrees=int(aux[8])
					m.offset=0
					m.unitsRange=0
					m.name=aux[0]
					self.motores[aux[0]]=m
					statem=MotorState()

					statem.p=0
					statem.v=0
					statem.temperature=0
					statem.isMoving=False
					statem.pos=m.zeroPos
					statem.vel=0.1
					statem.power=1.
					statem.timeStamp=""

					self.mstateMap[m.name]=statem
				for j in self.motores:
					print j," = ",self.motores[j]
		except:
			traceback.print_exc()
			print "Error reading config params"
		return True

	def configGetString(self, cadena, valdef="default"):
		value = valdef
		with open("config","r") as f:
			for linea in f.readlines():
				if linea[0:1]!="#":
					if linea[0:len(cadena)]==cadena:
						aux = linea.split("=")
						val = aux[1]
						if val[0:1]==" ":
							value=val[1:len(val)]
						else :
							value=val[0:len(val)]
						value = value.replace("\n","")
						if "#" in value:
							pos=value.find("#")
							value.replace(value[pos],"")
						break
		f.close()
		return value
	def mapear(self, x, in_min, in_max, out_min, out_max):
		return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min

	@QtCore.Slot()
	def compute(self):
		x=len(self.lisPos)
		y=len(self.lisVel)
		if(x!=0 or y!=0):
			self.timer.timeout=0
			try:
				if(x!=0):
					m=self.lisPos.popleft()
					busId=self.motores[m.name].busId

					pos=mapear(m.position, -1,1, 0,1023)
					dynamixel.set_position(self.ser, busId, pos)
					dynamixel.send_action_packet(self.ser)
				if(y!=0):
					m=self.lisVel.popleft()
					busId=self.motores[m.name].busId

					vel=mapear(m.velocity, 0,1, 0,1023)
					dynamixel.set_velocity(self.ser, busId, vel)
					dynamixel.send_action_packet(self.ser)
			except Ice.Exception, e:
				traceback.print_exc()
				print e
		else:
			self.timer.timeout=100
		self.mutmState.lock(self,self.updateState)





		print 'SpecificWorker.compute...'
		"""
		m=MotorState()
		try:
			m.isMoving=dynamixel.get_is_moving(self.ser,self.motores[motor].busId)
			pos=dynamixel.get_position(self.ser,self.motores[motor].busId)
			m.pos=(pos)*(1+1)/(1023)-1
			#m.p
			#m.v
			#m.temperature
			#m.power
			#m.timeStamp
			#m.vel
		except Ice.Exception, e:
			traceback.print_exc()
			print e
		"""

		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True

	def updateState(self):
		print "abasdfasfasdfasdf"
		for mp in self.motores:
			try:
				mpar=self.motores[mp]
				m=self.mstateMap[mp]
				m.isMoving=dynamixel.get_is_moving(self.ser,mpar.busId)
				pos=dynamixel.get_position(self.ser,mpar.busId)
				m.pos=mapear(pos, 0,1023, -1,1)
			except Ice.Exception, e:
				traceback.print_exc()
				print e
		pass
	#
	# getAllMotorParams
	#
	def getAllMotorParams(self):
		ret =MotorParamsList()
		for motorparams in self.motores:
			ret+=motorparams
		return ret


	#
	# getAllMotorState
	#
	def getAllMotorState(self):
		return self.mstateMap


	#
	# getMotorParams
	#
	def getMotorParams(self, motor):
		ret = self.motores[motor]
		return ret


	#
	# getMotorState
	#
	def getMotorState(self, motor):
		ret=self.mstateMap[motor]
		return ret


	#
	# setSyncVelocity
	#
	def setSyncVelocity(self, listGoals):
		for goal in listGoals:
			self.lisVel.append(goal)
		pass


	#
	# setZeroPos
	#
	def setZeroPos(self, name):
		goal=MotorGoalPosition()
		goal.name=name
		goal.position=self.motores[name].zeroPos
		goal.maxSpeed=0.1
		self.lisPos.append(goal)
		pass


	#
	# getBusParams
	#
	def getBusParams(self):
		return self.busParams


	#
	# setSyncZeroPos
	#
	def setSyncZeroPos(self):
		for m in self.motores:
			goal=MotorGoalPosition()
			goal.name=m.name
			goal.position=self.motores[m.name].zeroPos
			goal.maxSpeed=0.1
			self.lisPos.append(goal)
		pass


	#
	# setSyncPosition
	#
	def setSyncPosition(self, listGoals):
		for goal in listGoals:
			self.lisPos.append(goal)
		pass


	#
	# getMotorStateMap
	#
	def getMotorStateMap(self, mList):
		ret = MotorStateMap()
		for name in mList:
			ret[name]=self.mstateMap[name]
		return ret


	#
	# setPosition
	#
	def setPosition(self, goal):
		self.lisPos.append(goal)
		pass


	#
	# setVelocity
	#
	def setVelocity(self, goal):
		self.lisVel.append(goal)
		pass