#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################
# Adaptado de la práctica 3 de TSI #
####################################

from math import sqrt, atan2, sin, cos, pi
import sys
import roslib
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from posicion import Posicion


scan = []
obstaculos = []
odometria = Posicion(0,0,0)
odometria.yaw = 0
meta = None

FRONTAL = 0
TRASERO = 180
IZQ_FRONTAL = 60
IZQ_TRASERO = 120
DER_FRONTAL = 300
DER_TRASERO = 240

angulos = [i*60 for i in range(6)]
tolerancia = 0.009		#Aumentar
V_LINEAL_CTE = 0.25
V_LINEAL_MIN = 0.07
V_ANGULAR_CTE = 0.5

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
marker_pub = rospy.Publisher('/potencial', Marker)

potencial = Marker()
potencial.type = 0 		#Flecha
potencial.action = 0 	#Añadir/modificar
potencial.scale.y = 0.1 #Ancho
potencial.scale.z = 0.1 #Alto
potencial.color.r = 0
potencial.color.g = 0
potencial.color.b = 1
potencial.color.a = 1
potencial.header.frame_id="base_link"


def distancia(src, dst):
	return sqrt((src.x - dst.x) * (src.x - dst.x) + (src.y - dst.y) * (src.y - dst.y))

def signo(a):
	if a == 0:
		return 1
	else:
		return a/abs(a)

def campoAtractivo(meta, odometria):
	radio		= 0.15		#Radio del punto objetivo
	extension 	= 0.5 		#Extensión del campo
	intensidad 	= 0.15		#Fuerza del campo
	
	x = 0
	y = 0
	d = distancia(meta, odometria)
	#print "Yaw\t\t%f"%odometria.yaw

	theta = atan2(meta.y - odometria.y, meta.x - odometria.x) - odometria.yaw
	#print "Distandia:\t%f"%d
	#print "Theta:\t\t%f"%theta
	# Si estamos en el punto paramos
	if d-radio < tolerancia:
		#print "a"
		return Posicion(x,y,0)

	if (radio < d) and (d <= (extension + radio)):
		x = intensidad *(d - radio)*cos(theta)
		y = intensidad *(d - radio)*sin(theta)
		#print "b"
		return Posicion(x,y,0)

	if d > (extension + radio):
		x = intensidad*extension*cos(theta)
		y = intensidad*extension*sin(theta)
		#print "c"
		return Posicion(x,y,0)


def campoRepulsivo(obstaculo, odometria):
	radio		= 0.11		#Radio del punto objetivo
	extension 	= 0.3 		#Extensión del campo
	intensidad 	= 0.4 		#Fuerza del campo

	x = 0
	y = 0

	d = distancia(obstaculo, odometria)
	theta = atan2(obstaculo.y - odometria.y, obstaculo.x - odometria.x) - odometria.yaw

	if d < radio:
		x = -signo(cos(theta))*intensidad
		y = -signo(sin(theta))*intensidad
		return Posicion(x,y,0)

	if (radio < d) and (d <= extension + radio):
		x = -intensidad *(extension + radio - d)*cos(theta)
		y = -intensidad *(extension + radio - d)*sin(theta)
		return Posicion(x,y,0)

	if (d > (extension+ radio)):
		return Posicion(x,y,0)


def normalize(angle):
	angle = (angle + pi)%(2*pi)
	if (angle < 0):
		angle += 2*pi
	return angle - pi


def calculavAngular(v, odometria):
	angulo = normalize(atan2(v.y, v.x))
	#angulo = normalize(angulo)#-odometria.yaw)
	# print "Vector:\t[%f,%f]"%(v.x,v.y)
	# print "Diferencia:\t%f"%angulo
	# print "Angulo:\t%f"%angulo
	# print "Yaw:\t%f"%odometria.yaw

	if ((0 <= angulo) and (angulo <= pi)):
		if (angulo > V_ANGULAR_CTE):
			v_angular = V_ANGULAR_CTE
		else:
			if (angulo < tolerancia):
				v_angular = 0
			else:
				v_angular = angulo
			#v_angular = (angulo < tolerancia)? 0: angulo
	else:
		if angulo < -1*V_ANGULAR_CTE:
			v_angular = -1*V_ANGULAR_CTE
		else:
			if angulo > -1*tolerancia:
				v_angular = 0
			else:
				v_angular = angulo
			# v_angular = (angulo > (-1)*tolerancia)? 0: angulo
	return v_angular

def calculavLineal(total):
	angulo = normalize(atan2(total.y, total.x))# - odometria.yaw)
	v_lineal = sqrt(total.x*total.x + total.y*total.y)*2

	if v_lineal < V_LINEAL_MIN:
		v_lineal = V_LINEAL_MIN
	elif v_lineal > V_LINEAL_CTE:
		v_lineal = V_LINEAL_CTE

	if (-pi/2 <= angulo <= pi/2):
		v_lineal *= 1
	else:
		v_lineal *= -1


	return v_lineal


def main():
	global meta

	rospy.init_node('potencial')
	rospy.loginfo("Nodo iniciado")

	rospy.Subscriber("/arduino/sensor/multi_sonar", LaserScan, sonarCallback)
	#rospy.Subscriber("/base_scan", LaserScan, sonarCallback)

	rospy.Subscriber("/odom", Odometry, odomCallback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
	r = rospy.Rate(10)	#10Hz, la misma frecuencia que le odometria

	while not rospy.is_shutdown():
		# Esperamos hasta que se haya publicado un objetivo
		rospy.loginfo("Esperando publicación de objetivo")
		while not meta:
			pass

		rospy.loginfo("Yendo a objetivo")
		# Mientras que no hayamos llegado a la meta
		#while odometria != meta:
		atractivo = campoAtractivo(meta, odometria)


		while atractivo.x != 0 or atractivo.y != 0:
			# Generar campo atractivo
			atractivo = campoAtractivo(meta, odometria)
			# Generar campos repulsivos
			repulsivo = Posicion(0,0,0)
			for o in obstaculos:
				repulsivo = repulsivo + campoRepulsivo(o, odometria)

			#print repulsivo
			total = repulsivo + atractivo
			if total: 	#total nunća debería ser nulo, pero por si acaso
				#total= Posicion(0.1,0,0)
				potencial.scale.x = sqrt(total.x*total.x + total.y*total.y)*10
				#print potencial.scale.x
				angulo_potencial = atan2(total.y,total.x)

				quaternion_pot = tf.transformations.quaternion_from_euler(0,0,angulo_potencial)
				potencial.pose.orientation.x = quaternion_pot[0]
				potencial.pose.orientation.y = quaternion_pot[1]
				potencial.pose.orientation.z = quaternion_pot[2]
				potencial.pose.orientation.w = quaternion_pot[3]

				#potencial.pose.orientation.x = total.x
				#potencial.pose.orientation.y = total.y
				potencial.header.stamp = rospy.Time.now()
				# Convertir vector a twist

				v_lineal = calculavLineal(total)
				v_angular = calculavAngular(total, odometria)

				twist = Twist()
				twist.linear.x = v_lineal
				twist.angular.z = v_angular
				rospy.loginfo("Odometria:\t[%f, %f]" %(odometria.x, odometria.y))
				rospy.loginfo("Meta:\t[%f, %f]" %(meta.x, meta.y))

				# Publicar twist
				cmd_vel_pub.publish(twist)

				marker_pub.publish(potencial)
			r.sleep()

		rospy.loginfo("Objetivo alcanzado")
		meta = None

def sonarCallback(laser):
	global scan
	global obstaculos
	obstaculos = []
	scan = [laser.ranges[FRONTAL], laser.ranges[IZQ_FRONTAL], laser.ranges[IZQ_TRASERO], 
		laser.ranges[TRASERO], laser.ranges[DER_TRASERO], laser.ranges[DER_FRONTAL]]

	#print scan
	for i,s in enumerate(scan):
		if s >= 0.1 and s < 1:
			x = odometria.x + s*cos(odometria.yaw + angulos[i])
			y = odometria.y + s*sin(odometria.yaw + angulos[i])
			obstaculos.append(Posicion(x,y,0))


def odomCallback(odom):
	global odometria
	global potencial
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	z = odom.pose.pose.position.z

	quaternion = (
		odom.pose.pose.orientation.x,
		odom.pose.pose.orientation.y,
		odom.pose.pose.orientation.z,
		odom.pose.pose.orientation.w,
		)
	odometria = Posicion(x,y,tf.transformations.euler_from_quaternion(quaternion)[2])
	#odometria.yaw =  #atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z)
	#potencial.pose.position = odom.pose.pose.position

def goalCallback(goal):
	global meta
	x = goal.pose.position.x
	y = goal.pose.position.y
	meta = Posicion(x,y,0)

main()