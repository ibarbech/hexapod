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

import sys, os, Ice, traceback, time

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

from pydynamixel import dynamixel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.Device=""
		self.BaudRate=0
		self.BasicPeriod=0
		self.SDK=True
		self.NumMotors=0
		self.motores={}
		self.setParams()
		try:
			self.ser = dynamixel.get_serial_for_url(self.Device)
		except Exception as e:
			print('Unable to move to desired position.')
        	print(e)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self):
		try:
			name="Dynamixel"
			self.NumMotors=int(self.configGetString("Dynamixel.NumMotors","0"))

			for i in range(self.NumMotors):
				params=self.configGetString(name+".Params_"+str(i), "default")
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
				#diccionario={'BusId':int(aux[1]), 'InvertedSign':invert, 'MinPos':float(aux[3]), 'MaxPos':float(aux[4]), 'zero':int(aux[5]), 'maxVel':int(aux[6]), 'steps':int(aux[7]), 'MaxDegrees':int(aux[8])}
				self.motores[aux[0]]=m
			for j in self.motores:
				print j," = ",self.motores[j]
		except:
			traceback.print_exc()
			print "Error reading config params"
		return True

	def configGetString(self, cadena, valdef):
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

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True


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
		#
		# YOUR CODE HERE
		#
		mstateMap = MotorStateMap()


		return mstateMap


	#
	# getMotorParams
	#
	def getMotorParams(self, motor):
		ret = self.motores[motor]
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# getMotorState
	#
	def getMotorState(self, motor):
		ret = MotorState()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# setSyncVelocity
	#
	def setSyncVelocity(self, listGoals):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# setZeroPos
	#
	def setZeroPos(self, name):
		busid = self.motores[name].busId
		dynamixel.init(self.ser, busid)
		dynamixel.set_position(self.ser, busid, 512)
		dynamixel.send_action_packet(self.ser)
		pass


	#
	# getBusParams
	#
	def getBusParams(self):
		ret = BusParams()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# setSyncZeroPos
	#
	def setSyncZeroPos(self):
		for motorparams in self.motores:
			busid = motorparams.busId
			dynamixel.init(self.ser, busid)
			dynamixel.set_position(self.ser, busid, 512)
			dynamixel.send_action_packet(self.ser)
		pass


	#
	# setSyncPosition
	#
	def setSyncPosition(self, listGoals):
		for goal in listGoals:
			busid = self.motores[goal.name].busId
			dynamixel.init(self.ser, busid)
			position=(goal.position+1)*(1023)/(1+1)
			dynamixel.set_position(self.ser, busid, position)
			dynamixel.send_action_packet(self.ser)
		pass


	#
	# getMotorStateMap
	#
	def getMotorStateMap(self, mList):
		ret = MotorStateMap()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# setPosition
	#
	def setPosition(self, goal):
		busid = self.motores[goal.name].busId
		dynamixel.init(self.ser, busid)
		position=(goal.position+1)*(1023)/(1+1)
		dynamixel.set_position(self.ser, busid, position)
		dynamixel.send_action_packet(self.ser)
		pass


	#
	# setVelocity
	#
	def setVelocity(self, goal):
		#
		# YOUR CODE HERE
		#
		pass





