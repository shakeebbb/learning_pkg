#!/usr/bin/env python

import rospy
import numpy
import random
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from learning_pkg.msg import Learn
from rospy.numpy_msg import numpy_msg

from keras.models import Sequential
from keras.layers import Dense, Activation

comm_msg = Learn()
comm_msg.instruction = "py2c:waiting4action?"

seed = 7
numpy.random.seed(seed)

gamma = 1  # discount factor
	
nFeatures = 640

action_dim = 4

# Model 0
model0 = Sequential()
model0.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model0.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model0.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model0.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 1
model1 = Sequential()
model1.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model1.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model1.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model1.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 2
model2 = Sequential()
model2.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model2.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model2.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model2.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

print("NN model initialized ...")

def commCallback(msg):

	global scan
	global commPub
	try:
		scan
	except NameError:
		print "Waiting for sensor information..."
		return
	
	if msg.instruction == "c2py:waiting4action":
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
		
		epsilon = numpy.random.binomial(n=1, p=0.8, size=None)
	
		if epsilon == 0: # explore random action
			msg.instruction = "py2c:check4action"
			msg.action = random.randint(0,action_dim)
			msg.state = scan
			msg.reward = float('nan')
			print("sending action 0")
			commPub.publish(msg)
		
		else: # choose the best action
			
			#print(scan.shape)

			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			temp_array = [model0_out, model0_out, model0_out]
			
			msg.instruction = "py2c:check4action"
			msg.action = temp_array.index(max(temp_array))
			#msg.state = scan
			msg.reward = float('nan')
			print("sending action 1")
			commPub.publish(msg)
		
	elif msg.instruction == "c2py:check4reward":
	
		#model0_out = model0.predict(msg.state)
		#model1_out = model1.predict(msg.state)
		#model2_out = model2.predict(msg.state)
		
		Q_star = max(model0_out, model0_out, model0_out)
		
		Q = msg.reward + gamma*(Q_star)
		
		#global action
		if msg.action == 0:
			model0.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		elif msg.action == 1:
			model1.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		elif msg.action == 2:
			model2.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		
		#Publish Instruction / Acknowledgement
		msg.instruction = "py2c:waiting4action?"
		commPub.publish(msg)
		
def scanCallback(msg):
	
	global scan	
	#scan = numpy.transpose(numpy.reshape(numpy.asarray(msg.ranges), (len(msg.ranges),1)));

	scan = numpy.transpose(msg.ranges.reshape((640, 1)))
	#print(scan.shape)
	#dataset = numpy.loadtxt("pima-indians-diabetes.csv", delimiter=",")
	#scan = dataset[:,0:4]

def main ():

	rospy.init_node('learn_node', anonymous=True)
	
	global commPub
	
	commPub = rospy.Publisher("local_communication", Learn, queue_size=10)
	
	rospy.Subscriber("local_communication", numpy_msg(Learn), commCallback)
	rospy.Subscriber("scan", numpy_msg(LaserScan), scanCallback)
	
	rate = rospy.Rate(5) # 10hz
	
	global bugHim
	
	while not rospy.is_shutdown():
		if comm_msg.instruction == "py2c:waiting4action?":
			commPub.publish(comm_msg)
		rate.sleep()
		#rospy.spinOnce()
	
if __name__ == '__main__':
	main()

