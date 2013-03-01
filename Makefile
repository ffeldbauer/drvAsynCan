# Makefile for Asyn drvAsynRPiCan support
#
# Created by florian on Thu Feb 28 14:19:22 2013
# Based on the Asyn app template

TOP = .
include $(TOP)/configure/CONFIG

DIRS := configure
DIRS += $(wildcard *[Ss]up)
DIRS += $(wildcard *[Aa]pp)
DIRS += $(wildcard ioc[Bb]oot)

include $(TOP)/configure/RULES_TOP
