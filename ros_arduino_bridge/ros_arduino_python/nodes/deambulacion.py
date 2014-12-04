#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

FRONTAL = 0
TRASERO = 180
IZQ_FRONTAL = 60
IZQ_TRASERO = 120
DER_FRONTAL = 300
DER_TRASERO = 240

V_LIN = 0.2
V_ANG = 0.5
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)

def main():
	rospy.init_node('deambulacion')
	rospy.loginfo("Deambulación iniciada")

	rospy.Subscriber("/arduino/sensor/multi_sonar", LaserScan, Callback)

	while not rospy.is_shutdown():
		pass


# Función que determina la velodidad lineal.
# Si distancia < 0.2; v = 0
# Si distancia > 0.4; v = V_LIN
def velocidad_lineal(distancia):

	v = 0.5*distancia -0.1
	if v < 0:
		v = 0
	elif v > V_LIN:
		v = V_LIN
	return v

# Función que determina la velodidad angular.
def velocidad_angular(distancia):
	# Recta que pasa por (0.3,0.5) y (0.6,0)
	v = 1 -(5/3)*distancia
	return v


def Callback(laser):
	v_angular = 0
	ranges = list(laser.ranges)
	# for i in range(len(ranges)):
	# 	# Los sonar devuelven 0.1 cuando no son capaces de medir la distancia
	# 	# Esta conversión da algún problema cuando ya tenemos algo completamente encima, pero en general compensa
	# 	if ranges[i] == 0.1:
	# 		ranges[i] = float("inf")


	# Obstáculos a la izquierda: mínimo de los dos sensores laterales izquierdos
	izquierda = min(ranges[IZQ_TRASERO], ranges[IZQ_FRONTAL])

	# Obstáculos a la derecha: mínimo de los dos sensores laterales derechos
	derecha = min(ranges[DER_TRASERO], ranges[DER_FRONTAL])

	# Si no hay nada cercano al frente avanzamos
	if ranges[0] > 0.3:
		v_lineal = velocidad_lineal(ranges[FRONTAL])
	else:
		# En otro caso, si hay algo más cercano aún por detrás, avanzamos
		if ranges[0] >= ranges[TRASERO]:	
			v_lineal = velocidad_lineal(ranges[FRONTAL])
		# Si hay hueco por detrás retrocedemos (si es posible, girando para evitar estancarnos)
		else:	
			v_lineal = -velocidad_lineal(ranges[TRASERO])

			if izquierda > 0.3 or derecha > 0.3:
				if izquierda >= derecha:
					v_angular = V_ANG
				else:
					v_angular = -V_ANG

	# Control de obstáculos por los laterales
	if izquierda < 0.3:
		v_angular += -velocidad_angular(izquierda)

	if derecha < 0.3:
		v_angular += velocidad_angular(derecha)

	twist = Twist()

	rospy.loginfo("v_lineal = %f"%(v_lineal))
	rospy.loginfo("v_angular = %f"%(v_angular))
	twist.linear.x = v_lineal
	twist.angular.z = v_angular
	cmd_vel_pub.publish(twist)

main()


