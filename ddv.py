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


###################################################
# Danny's Digital Voice                           #
###################################################
# Version:     v0.2.0                             #
###################################################


from datetime import datetime
from ddp import *
from Queue import Queue

import os
import subprocess
import threading


###########
# Classes #
###########
class DDV(DDP):
	def __init__(self, modem_port = "/dev/ttyu0", modem_speed = 9600, modem_bits = 1, modem_parity = "N", modem_stopbits = 1, ddv_retries = 1, ddv_data_length = 1024, ddv_tx_wait = 0.25, ddv_rx_wait = 0.1, ddv_timeout = 10., ddv_tx_hangtime = 0.1, ddv_specification = 1, ddv_codec = 1, ddvrx_thread_disable = False, ddv_dsp_play_device = "/dev/dsp", ddv_dsp_record_device = "/dev/dsp", ddv_disable_ec = False, ddv_disable_crypto = False, ddv_allow_unsigned_packets = False, ddv_application = "DDV", ddv_debug_mode = False):
		self.ddv_dsp_play = None
		self.ddv_dsp_record = None
		self.ddv_enabled = False
		self.ddvrx_alive = True
		self.ddvrx_buffer = Queue()
		self.ddvrx_play_thread = None
		self.ddvrx_thread = None
		self.ddvrx_thread_disable = ddvrx_thread_disable
		self.ddvtx_alive = True
		self.ddvtx_buffer = Queue()
		self.ddvtx_buffer_thread = None
		self.ddvtx_thread = None
		
		self.CODEC2_BITRATE = 1200 # 2400/1400/1200
		
		self.DDV_AUDIO_CHANNELS = 1
		self.DDV_AUDIO_SAMPLERATE = 8000
		self.DDV_AUDIO_SPS = (self.DDV_AUDIO_CHANNELS * self.DDV_AUDIO_SAMPLERATE) * 2 # 2x for 16bit
		self.DDV_CODEC_CODEC2 = 1
		self.DDV_CODEC_OGG = 2
		self.DDV_CODEC_SPEEX = 0
		self.DDV_CODEC = ddv_codec
		self.DDV_DSP_PLAY_DEVICE = ddv_dsp_play_device
		self.DDV_DSP_RECORD_DEVICE = ddv_dsp_record_device
		self.DDV_VERSION = "0.2.0"
		
		
		# Now get the base class initialised
		DDP.__init__(self, hostname = modem_port, port = "%s/%s/%s/%s" % (modem_speed, modem_bits, modem_parity, modem_stopbits), data_mode = "EXTENSION", data_length = ddv_data_length, tx_wait = 0.1, rx_wait = 0.01, timeout = ddv_timeout, ack_timeout = 15., tx_hangtime = ddv_tx_hangtime, specification = ddv_specification, extension_init = self.setupDDV, disable_ec = ddv_disable_ec, disable_crypto = ddv_disable_crypto, allow_unsigned_packets = ddv_allow_unsigned_packets, application = ddv_application, ignore_broadcast_packets = True, repeater_mode = False, colour_logging = True, logger_name = "DDV", debug_mode = ddv_debug_mode)
		
		
		# Show any information we need to at this point
		if self.DDV_CODEC == self.DDV_CODEC_SPEEX:
			self.log.info("Using speex for the DV codec.")
			
		elif self.DDV_CODEC == self.DDV_CODEC_CODEC2:
			self.log.info("Using Codec2 @ %d bits/s for the DV codec." % self.CODEC2_BITRATE)
			
		elif self.DDV_CODEC == self.DDV_CODEC_OGG:
			self.log.info("Using OGG for the DV codec.")
			self.log.warn("The OGG codec is only to be used for local testing ONLY - DO NOT USE ON-AIR.")
			
		else:
			self.log.warn("Unknown codec type for DV, please change.")
	
	def ddvDecodeCodec(self, data, unlink = True):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			d = None
			
			if self.DDV_CODEC == self.DDV_CODEC_SPEEX:
				d = subprocess.Popen(["speexdec", "--mono", "--rate", str(self.DDV_AUDIO_SAMPLERATE), "-", "-"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			elif self.DDV_CODEC == self.DDV_CODEC_CODEC2:
				d = subprocess.Popen(["./c2dec", str(self.CODEC2_BITRATE), "-", "-"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			elif self.DDV_CODEC == self.DDV_CODEC_OGG:
				d = subprocess.Popen(["oggdec", "--quiet", "--raw", "--bits", "16", "--endianness", "0", "--output", "-", "-"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			else:
				self.log.error("Unknown codec type, cannot decode.")
			
			raw_data = d.communicate(data)
			
			d = None
			
			
			return raw_data[0]
	
	def ddvEncodeCodec(self, data):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			d = None
			
			if self.DDV_CODEC == self.DDV_CODEC_SPEEX:
				d = subprocess.Popen(["speexenc", "--narrowband", "--abr", "3600", "--dtx", "--comp", "4", "--nframes", "1", "--rate", str(self.DDV_AUDIO_SAMPLERATE), "--le", "--16bit", "-", "-"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			elif self.DDV_CODEC == self.DDV_CODEC_CODEC2:
				d = subprocess.Popen(["./c2enc", str(self.CODEC2_BITRATE), "-", "-"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			elif self.DDV_CODEC == self.DDV_CODEC_OGG:
				d = subprocess.Popen(["oggenc", "--raw", "--raw-bits", "16", "--raw-chan", str(self.DDV_AUDIO_CHANNELS), "--raw-rate", str(self.DDV_AUDIO_SAMPLERATE), "--raw-endianness", "0", "-q", "5", "-", "--output", "-", "--discard-comments"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
				
			else:
				self.log.error("Unknown codec type, cannot encode.")
			
			
			raw_data = d.communicate(data)
			
			d = None
			
			
			return raw_data[0]
	
	def ddvPlayThread(self):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			while self.ddvrx_alive:
				# Play buffer until it's done
				try:
					if not self.ddvrx_buffer.empty():
						d = self.ddvrx_buffer.get()
					
						if self.DEBUG_MODE:
							self.log.info("Playing audio...")
						
						self.ddv_dsp_play.write(d)
						
						if self.DEBUG_MODE:
							self.log.info("Finished playing audio.")
						
					else:
						time.sleep(0.01)
					
				except Exception, ex:
					self.log.fatal(str(ex))
	
	def ddvRecord(self):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		self.ddvtx_alive = True
		
		self.ddvtx_thread = threading.Thread(target = self.ddvRecordThread)
		self.ddvtx_thread.setDaemon(1)
		self.ddvtx_thread.start()
		
		self.ddvtx_buffer_thread = threading.Thread(target = self.ddvTXBuffer)
		self.ddvtx_buffer_thread.setDaemon(1)
		self.ddvtx_buffer_thread.start()
	
	def ddvRecordThread(self):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			while self.ddvtx_alive:
				try:
					self.ddvtx_buffer.put(self.ddv_dsp_record.read(self.DDV_AUDIO_SPS))
					
				except Exception, ex:
					self.log.fatal(str(ex))
	
	def ddvRXLoop(self):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			while self.ddvrx_alive:
				try:
					data = self.receiveDataFromAny("DDVQSO")
					
					if data is not None:
						# Check the flags
						d = data[0]
						packet = data[1]
						
						raw = self.ddvDecodeCodec(d)
						self.ddvrx_buffer.put(raw)
						
					else:
						time.sleep(0.01)
					
				except Exception, ex:
					self.log.fatal(str(ex))
	
	def ddvStopRecord(self):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		# Ensure we've got the rig off key
		self.ptt(False)
		
		self.ddvtx_alive = False
	
	def ddvTXBuffer(self):
		lock = threading.Lock()
		
		with lock:
			while self.ddvtx_alive:
				# Ensure we've got enough to transmit
				blen = self.ddvtx_buffer.qsize()
				
				if blen >= 1:
					# Save the raw data ready to pass to the encoder
					d = self.ddvtx_buffer.get()
					
					# Now encode it
					e = self.ddvEncodeCodec(d)
					
					if self.transmitData(self.CALLSIGN, "", "DDVQSO", ":", ":", e) == False:
						self.log.info("Unable to transmit packet.")
					
					f.close()
					f = None
					
				else:
					time.sleep(0.01)
	
	def dispose(self):
		if self.ddv_enabled:
			if self.ddvrx_alive:
				self.ddvrx_alive = False
				
			self.ddv_dsp_play.close()
			self.ddv_dsp_record.close()
			
			self.ddv_dsp_play = None
			self.ddv_dsp_record = None
		
		DDP.dispose(self)
	
	def log(self, level, message):
		self.logger.log(level, message)
	
	def setupDDV(self, device = "/dev/ttyu0", port = "9600/8/N/1", timeout = 60.):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		# These need to be changed first
		self.BACKEND = "DDV"
		self.PROTOCOL_FOOTER = "*DDVF*"
		self.PROTOCOL_HEADER = "*DDVH*"
		
		self.setupRS232(device, port, timeout)
		
		
		# Fudge some settings for improved performance
		self.DATA_LENGTH = 16384
		self.MAX_RETRIES = 1
		self.TX_HANGTIME = 0.
		self.TX_WAIT = 0.
		
		
		# Ensure we can get access to the soundcard, otherwise disable DDV mode
		self.ddv_enabled = False
		
		try:
			import ossaudiodev
			
			
			if self.DEBUG_MODE:
				self.log.info("Setting up DSP device \"%s\" for recording..." % self.DDV_DSP_RECORD_DEVICE)
			
			self.ddv_dsp_record = ossaudiodev.open(self.DDV_DSP_RECORD_DEVICE, "r")
			self.ddv_dsp_record.setfmt(ossaudiodev.AFMT_S16_LE)
			self.ddv_dsp_record.channels(self.DDV_AUDIO_CHANNELS)
			self.ddv_dsp_record.speed(self.DDV_AUDIO_SAMPLERATE)
			
			
			if self.DEBUG_MODE:
				self.log.info("Setting up DSP device \"%s\" for playback..." % self.DDV_DSP_PLAY_DEVICE)
			
			self.ddv_dsp_play = ossaudiodev.open(self.DDV_DSP_PLAY_DEVICE, "w")
			self.ddv_dsp_play.setfmt(ossaudiodev.AFMT_S16_LE)
			self.ddv_dsp_play.channels(self.DDV_AUDIO_CHANNELS)
			self.ddv_dsp_play.speed(self.DDV_AUDIO_SAMPLERATE)
			
			
			self.ddv_enabled = True
			self.log.info("OSS module imported and configured successfully.")
			
		except Exception, ex:
			self.ddv_enabled = False
			
			self.log.fatal(str(ex))
			self.log.info("Unable to import and configure the OSS module, DDV will be disabled.")
		
		
		# Setup anything else before firing the threads
		self.devnull = open(os.devnull, "wb")
		
		
		# Get the background rx thread going
		if self.ddv_enabled and not self.ddvrx_thread_disable:
			self.ddvrx_thread = threading.Thread(target = self.ddvRXLoop)
			self.ddvrx_thread.setDaemon(1)
			self.ddvrx_thread.start()
			
			self.ddvrx_play_thread = threading.Thread(target = self.ddvPlayThread)
			self.ddvrx_play_thread.setDaemon(1)
			self.ddvrx_play_thread.start()
	
	def transmitData(self, callsign_from, via, callsign_to, data):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		chunks = self.splitDataIntoChunks(data, self.DATA_LENGTH)
		
		
		# DDV always uses UDP
		compress = 1
		tcp = 0
		
		
		flags = Bits()
		flags.set(self.FLAG_TCP, tcp)
		flags.set(self.FLAG_COMPRESSION, compress)
		flags.set(self.FLAG_RETURN_DATA, 1)
		flags.set(self.FLAG_EC, int(self.EC_AVAILABLE))
		flags.set(self.FLAG_SYN, 1)
		
		
		self.ptt(True)
		
		for c in chunks:
			if self.DEBUG_MODE:
				self.log.info("Sending SYN...")
			
			self.transmitPacket(callsign_from, via, callsign_to, flags, c)
		
		return True
	
	def transmitRawPacket(self, packet):
		if self.DEBUG_MODE:
			self.log.info("Running...")
		
		
		lock = threading.Lock()
		
		with lock:
			s = bytearray()
			s.extend(str(packet))
			
			
			wrote = None
			
			try:
				if self.DEBUG_MODE:
					self.log.info("Transmitting raw packet %s (%d bytes)..." % (repr(packet), len(str(packet))))
				
				
				# Time how long it takes to write the data in-case the OS caches the serial data
				starttime = time.time()
				
				w = self.serial.write(str(s))
				self.serial.flush()
				
				finishtime = time.time()
				
				
				dlen = float(len(str(s)))
				timeneeded = (dlen / self.SERIAL_BPS)
				timetaken = (finishtime - starttime)
				
				if self.DEBUG_MODE:
					self.log.info("Data length: %d, BPS: %d, Time required: %.5f, Time taken: %.5f" % (dlen, self.SERIAL_BPS, timeneeded, timetaken))
				
				
				p = round(timeneeded - timetaken, 3)
				
				if p > 0.:
					if self.DEBUG_MODE:
						self.log.warn("Waiting for %.3f seconds for data to flush..." % p)
					
					time.sleep(p)
				
				
				if w is not None:
					if wrote is None:
						wrote = 0
					
					wrote += w
				
			except Exception, ex:
				if self.DEBUG_MODE:
					self.log.fatal(str(ex))
			
			
			if wrote is not None:
				
				if wrote <> len(s):
					if self.DEBUG_MODE:
						self.log.warn("%d/%d bytes where written to the serial port." % (wrote, len(s)))
