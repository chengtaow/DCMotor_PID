import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import numpy as np
import time
import random
import cv2

class Ferriclean:
	def __init__(self):
		#init signal for motor dirction
		GPIO.setup("P8_7", GPIO.OUT)
		GPIO.setup("P8_8", GPIO.OUT)
		GPIO.setup("P8_9", GPIO.OUT)
		GPIO.setup("P8_10", GPIO.OUT)
		#init signal for left, right brush
		GPIO.setup("P8_17", GPIO.OUT)
		GPIO.setup("P8_18", GPIO.OUT)
		#init signal for left, right encoder
		GPIO.setup("P9_15", GPIO.IN)
		GPIO.setup("P9_21", GPIO.IN)
		#set up ADC for IR sensor
		ADC.setup()
		#set up the camera and face cascade
		self.cam = cv2.VideoCapture(0)
		self.face_cascade = cv2.CascadeClassifier('/root/Ferriclean_Robot/haarcascade_frontalface_default.xml')
		self.face = False
		#the parameter of wheels direction and distance
		self.direction_right = True
		self.direction_left = True
		self.distance_right = 0
		self.distance_left = 0
		#the robot left and right wheels PWM duty
		self.lefd = 40
		self.rigd = 40
		#the robot desired speed, set point, unit: counts/ sec
		self.lefsp = 400
		self.rigsp = 400

	def read_IR(self):
		self.mid = ADC.read("P9_35")
        	self.lef = ADC.read("P9_33")
       		self.rig = ADC.read("P9_36")	
		self.mrig = ADC.read("P9_37")
		self.mlef = ADC.read("P9_39")
		self.value = [self.lef, self.mid, self.rig, self.mlef, self.mrig]

	def __leftf(self):
		# The left wheel go forward
		PWM.start("P8_13", self.lefd, 50, 0)
		GPIO.output("P8_7", GPIO.LOW)
		GPIO.output("P8_8", GPIO.HIGH)
	def __leftb(self):
		# The left wheel go back
		PWM.start("P8_13", self.lefd, 50, 0)
		GPIO.output("P8_7", GPIO.HIGH)
		GPIO.output("P8_8", GPIO.LOW)
	def __rigf(self):
		# The right wheel go forward
		PWM.start("P9_14", self.rigd, 50, 0)
		GPIO.output("P8_9", GPIO.HIGH)
        	GPIO.output("P8_10", GPIO.LOW)
	def __rigb(self):
		# The right wheel go back
		PWM.start("P9_14", self.rigd, 50, 0)
		GPIO.output("P8_9", GPIO.LOW)
        	GPIO.output("P8_10", GPIO.HIGH)

	def goforward(self):
		# The robot go forward
		self.direction_right = True
		self.direction_left = True
		self.__leftf()
		self.__rigf()

	def goback(self):
		# The robot go back
		self.direction_right = False
		self.direction_left = False
		self.lefd = 25
		self.rigd = 25
		self.__leftb()
		self.__rigb()

	def turnrig(self):
		# The robot turn right
		self.direction_right = False
		self.direction_left = True
		self.lefd = 25
		self.rigd = 25
		self.__leftf()
		self.__rigb()

	def turnleft(self):
		# The robot turn left
		self.direction_right = True
		self.direction_left = False
		self.lefd = 25
		self.rigd = 25
		self.__leftb()
		self.__rigf()

	def stop(self):
		# The robot stop
		self.lefd = 0
		self.rigd = 0
		self.__leftb()
		self.__rigb()

	def encoder_right(self,channel):
		# The call back function for right encoder
		if self.direction_right == True:
			self.distance_right += 1
		if self.direction_right == False:
			self.distance_right -= 1

	def encoder_left(self,channel):
		# The call back function for left encoder
		if self.direction_left == True:
			self.distance_left += 1
		if self.direction_left == False:
			self.distance_left -= 1

	def setupencoder(self):
		# Set up the interrupt of encoder
		GPIO.add_event_detect("P9_15", GPIO.BOTH, callback = self.encoder_right)
		GPIO.add_event_detect("P9_21", GPIO.BOTH, callback = self.encoder_left)

	def detectface(self):
		_, img = self.cam.read()
		res = cv2.resize(img,None,fx=0.4,fy=0.4,interpolation = cv2.INTER_CUBIC)
		gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
		for (x,y,w,h) in faces:
	              	cv2.rectangle(res,(x,y),(x+w,y+h),(255,0,0),2)
	       	print faces
	       	if type(faces).__module__ == np.__name__:
			self.stop()
			print 'find the face!'
			self.face = True

	def obstacle_avoidance(self):
		last_time = time.time()
		while True:
			# For the obstacle avoidance
			#self.detectface()
			#if self.face == True:
			#	break
			#else:
				print("--- %s seconds ---" % (time.time() - last_time))
				last_time = time.time()
				self.read_IR()		
				if self.lef > 0.35 and self.rig < 0.35:
					self.turnrig()
				elif self.rig > 0.35 and self.lef < 0.35:
					self.turnleft()
				elif self.lef > 0.3 and self.rig > 0.3 or self.mid > 0.3:
					ranturn = random.randint(1,4)
					if ranturn > 2:
						self.turnrig()
					else:
						self.turnleft()
					time.sleep(1)
				else:
					self.goforward()

				# Also control the brush
				if self.mlef > 0.4:
					GPIO.output("P8_17", GPIO.HIGH)
				else:
					GPIO.output("P8_17", GPIO.LOW)
				if self.mrig > 0.4:
					GPIO.output("P8_18", GPIO.HIGH)
				else:
					GPIO.output("P8_18", GPIO.LOW)

				# Print the distance
				#print self.distance_left, self.distance_right
				#print self.value
				#time.sleep(0.1)

	def go_straight(self):
		# self.leftd self.rigd is the duty cycle of the PWM signal
		last_time = time.time()
		initial_time = last_time
		last_left = self.distance_left
		last_right = self.distance_right
		# The sum error of left, right motor
		l_error_sum = 0
		r_error_sum = 0
		# The last speed of left, right motor
		l_last = 0
		r_last = 0
		# Different control gains
		kp = 0.07
		ki = 0
		kd = 0.03
		# Just use to break loop and get the test data
		i = 0 
		lspeed = []
		rspeed = []
		timetotal = []
		while True:
			self.goforward()
			time.sleep(0.1)
			time_use = time.time() - last_time
			left_go = self.distance_left - last_left
			right_go = self.distance_right - last_right
			last_left = self.distance_left
			last_right = self.distance_right
			last_time = time.time()
			timetotal.append(time.time() - initial_time)
			left_speed = left_go / time_use
			right_speed = right_go / time_use
			lspeed.append(left_speed)
			rspeed.append(right_speed)
			l_error = self.lefsp - left_speed
			r_error = self.rigsp - right_speed
			l_error_sum = l_error_sum + l_error
			r_error_sum = r_error_sum + r_error
			
			self.lefd = self.lefd + kp * l_error + ki * l_error_sum + kd * (l_last - left_speed)
			self.rigd = self.rigd + kp * r_error + ki * l_error_sum + kd * (r_last - right_speed)

			if self.lefd > 100:
				self.lefd = 100
			elif self.lefd < 0:
				self.lefd = 0

			if self.rigd > 100:
				self.rigd = 100
			elif self.rigd < 0:
				self.rigd = 0

			l_last = left_speed
			r_last = right_speed
			#print '--- %.2f seconds ---' % time_use
			print '--- left, right wheel speed: %.2f' % left_speed, ', %.2f counts/sec---' % right_speed
			print '--- left, right motor PWM duty cycle: %.2f' % self.lefd, ', %.2f ---' % self.rigd
			print ' '
			i = i + 1
			if i > 100:
				# We use these data to plot the result
				print lspeed
				print rspeed
				print timetotal
				break

	def __del__(self):
		GPIO.cleanup()
		PWM.stop("P8_13")
		PWM.stop("P9_14")
		PWM.cleanup()
		print 'Good Bye!'
