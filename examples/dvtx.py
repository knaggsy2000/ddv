#!/usr/bin/env python
# -*- coding: utf-8 -*-

#########################################################################
# Copyright/License Notice (BSD License)                                #
#########################################################################
#########################################################################
# Copyright (c) 2010-2012, Daniel Knaggs - 2E0DPK/M6DPK                 #
# All rights reserved.                                                  #
#                                                                       #
# Redistribution and use in source and binary forms, with or without    #
# modification, are permitted provided that the following conditions    #
# are met: -                                                            #
#                                                                       #
#   * Redistributions of source code must retain the above copyright    #
#     notice, this list of conditions and the following disclaimer.     #
#                                                                       #
#   * Redistributions in binary form must reproduce the above copyright #
#     notice, this list of conditions and the following disclaimer in   #
#     the documentation and/or other materials provided with the        #
#     distribution.                                                     #
#                                                                       #
#   * Neither the name of the author nor the names of its contributors  #
#     may be used to endorse or promote products derived from this      #
#     software without specific prior written permission.               #
#                                                                       #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR #
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, #
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      #
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, #
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  #
#########################################################################


from danlog import DanLog
from ddv import *
import os
import sys
import time
from xml.dom import minidom


#############
# Variables #
#############
client_callsign = ""

ddv = None

log = DanLog("DVTX")

rxalive = False


#############
# Constants #
#############
ALLOW_UNSIGNED_PACKETS = False

CODEC = 0

DEBUG_MODE = False
DISABLE_CRYPTO = False
DSP_PLAY_DEVICE = "/dev/dsp"
DSP_RECORD_DEVICE = "/dev/dsp"

MODEM_PORT = "/dev/ttyu0"
MODEM_SPEED = 9600
MODEM_BITS = 8
MODEM_PARITY = "N"
MODEM_STOPBITS = 1

SPECIFICATION = 1

XML_SETTINGS_FILE = "dvtx-settings.xml"


###############
# Subroutines #
###############
def cBool(value):
	if str(value).lower() == "false" or str(value) == "0":
		return False
		
	elif str(value).lower() == "true" or str(value) == "1":
		return True
		
	else:
		return False
	
def exitProgram():
	sys.exit(0)
	
def main():
	global client_callsign, ddv, rxalive, server_callsign
	
	
	log.info("""
#########################################################################
# Copyright/License Notice (BSD License)                                #
#########################################################################
#########################################################################
# Copyright (c) 2010-2012, Daniel Knaggs - 2E0DPK/M6DPK                 #
# All rights reserved.                                                  #
#                                                                       #
# Redistribution and use in source and binary forms, with or without    #
# modification, are permitted provided that the following conditions    #
# are met: -                                                            #
#                                                                       #
#   * Redistributions of source code must retain the above copyright    #
#     notice, this list of conditions and the following disclaimer.     #
#                                                                       #
#   * Redistributions in binary form must reproduce the above copyright #
#     notice, this list of conditions and the following disclaimer in   #
#     the documentation and/or other materials provided with the        #
#     distribution.                                                     #
#                                                                       #
#   * Neither the name of the author nor the names of its contributors  #
#     may be used to endorse or promote products derived from this      #
#     software without specific prior written permission.               #
#                                                                       #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR #
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, #
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      #
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, #
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  #
#########################################################################
""")
	log.info("")
	log.info("DV Transmit")
	log.info("===========")
	log.info("Checking settings...")
	
	if os.path.exists(XML_SETTINGS_FILE) == False:
		log.warn("The XML settings file doesn't exist, create one...")
		
		xmlTXSettingsWrite()
		
		
		log.info("The XML settings file has been created using the default settings.  Please edit it and restart DVTX once you're happy with the settings.")
		
		exitProgram()
		
	else:
		log.info("Reading XML settings...")
		
		xmlTXSettingsRead()
		
		# This will ensure it will have any new settings in
		if os.path.exists(XML_SETTINGS_FILE + ".bak"):
			os.unlink(XML_SETTINGS_FILE + ".bak")
			
		os.rename(XML_SETTINGS_FILE, XML_SETTINGS_FILE + ".bak")
		xmlTXSettingsWrite()
	
	log.info("Setting up DDV...")
	ddv = DDV(modem_port = MODEM_PORT, modem_speed = MODEM_SPEED, modem_bits = MODEM_BITS, modem_parity = MODEM_PARITY, modem_stopbits = MODEM_STOPBITS, ddv_timeout = 60., ddv_tx_hangtime = 0.1, ddv_codec = CODEC, ddv_specification = SPECIFICATION, ddvrx_thread_disable = True, ddv_dsp_play_device = DSP_PLAY_DEVICE, ddv_dsp_record_device = DSP_RECORD_DEVICE, ddv_disable_crypto = DISABLE_CRYPTO, ddv_allow_unsigned_packets = ALLOW_UNSIGNED_PACKETS, ddv_application = "DDV Voice Test", ddv_debug_mode = DEBUG_MODE)
	
	
	log.info("")
	
	while client_callsign == "":
		log.info("Please enter your callsign: ", newline = False)
		
		client_callsign = readInput().strip().upper()
		log.info("")
	
	ddv.setCallsign(client_callsign)
	
	
	try:
		log.info("Press ENTER to start recording...")
		readInput()
		ddv.ddvRecord()
		
		
		log.info("Press ENTER to stop recording, you should be transmitting in near real-time.")
		
		r = readInput()
		time.sleep(2.)
		ddv.ddvStopRecord()
		
	except KeyboardInterrupt:
		pass
		
	except Exception, ex:
		log.fatal(ex)
	
	
	log.info("Cleaning up...")
	ddv.dispose()
	ddv = None
	
	log.info("Exiting...")
	sys.exit(0)

def readInput():
	return sys.stdin.readline().replace("\r", "").replace("\n", "")

def xmlTXSettingsRead():
	global ALLOW_UNSIGNED_PACKETS, CODEC, DEBUG_MODE, DISABLE_CRYPTO, DSP_PLAY_DEVICE, DSP_RECORD_DEVICE, MODEM_BITS, MODEM_PARITY, MODEM_PORT, MODEM_SPEED, MODEM_STOPBITS, SPECIFICATION
	
	
	if os.path.exists(XML_SETTINGS_FILE):
		xmldoc = minidom.parse(XML_SETTINGS_FILE)
		
		myvars = xmldoc.getElementsByTagName("Setting")
		
		for var in myvars:
			for key in var.attributes.keys():
				val = str(var.attributes[key].value)
				
				# Now put the correct values to correct key
				if key == "ModemPort":
					MODEM_PORT = val
					
				elif key == "ModemSpeed":
					MODEM_SPEED = val
					
				elif key == "ModemBits":
					MODEM_BITS = val
					
				elif key == "ModemParity":
					MODEM_PARITY = val.upper()
					
				elif key == "ModemStopBits":
					MODEM_STOPBITS = val
					
				elif key == "DSPPlayDevice":
					DSP_PLAY_DEVICE = val
					
				elif key == "DSPRecordDevice":
					DSP_RECORD_DEVICE = val
					
				elif key == "Codec":
					CODEC = int(val)
				
				elif key == "Specification":
					SPECIFICATION = int(val)
					
				elif key == "AllowUnsignedPackets":
					ALLOW_UNSIGNED_PACKETS = cBool(val)
					
				elif key == "DisableCrypto":
					DISABLE_CRYPTO = cBool(val)
					
				elif key == "DebugMode":
					DEBUG_MODE = cBool(val)

def xmlTXSettingsWrite():
	if os.path.exists(XML_SETTINGS_FILE) == False:
		xmloutput = file(XML_SETTINGS_FILE, "w")
		
		
		xmldoc = minidom.Document()
		
		# Create header
		settings = xmldoc.createElement("DVTX")
		xmldoc.appendChild(settings)
		
		# Write each of the details one at a time, makes it easier for someone to alter the file using a text editor
		var = xmldoc.createElement("Setting")
		var.setAttribute("ModemPort", str(MODEM_PORT))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("ModemSpeed", str(MODEM_SPEED))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("ModemBits", str(MODEM_BITS))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("ModemParity", str(MODEM_PARITY))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("ModemStopBits", str(MODEM_STOPBITS))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("DSPPlayDevice", str(DSP_PLAY_DEVICE))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("DSPRecordDevice", str(DSP_RECORD_DEVICE))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("Codec", str(CODEC))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("Specification", str(SPECIFICATION))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("AllowUnsignedPackets", str(ALLOW_UNSIGNED_PACKETS))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("DisableCrypto", str(DISABLE_CRYPTO))
		settings.appendChild(var)
		
		var = xmldoc.createElement("Setting")
		var.setAttribute("DebugMode", str(DEBUG_MODE))
		settings.appendChild(var)
		
		
		# Finally, save to the file
		xmloutput.write(xmldoc.toprettyxml())
		xmloutput.close()


########
# Main #
########
if __name__ == "__main__":
	main()
