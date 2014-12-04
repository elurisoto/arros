#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Clase para simplificar el acceso a posiciones. También puede servir para almacenar vectores
class Posicion:
	def __init__(self, x, y, yaw):
		self.x = x
		self.y = y
		self.yaw = yaw

	#Compara si dos posiciones son MÁS O MENOS la misma. Deja un margen de error de 20 cm y 20º.
	# def __eq__(self, other):
	# 	return (other.x - 0.1 <= self.x <= other.x + 0.1) and \
	# 		(other.y - 0.1 <= self.y <= other.y + 0.1) and \
	# 		(other.w - 0.17 <= self.w <= other.w + 0.17)

	# !=
	def __ne__(self, other):
		return not(self == other)

	# Suma de vectores, ignorando la rotación
	def __add__(self, other):
		return Posicion(self.x + other.x, self.y + other.y, 0)

	def __str__(self):
		return ("[%f,%f,%f]"%(self.x, self.y, self.yaw))
		