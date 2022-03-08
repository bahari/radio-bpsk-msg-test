from gnuradio import analog
from gnuradio import audio
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.filter import pfb
from grc_gnuradio import blks2 as grc_blks2
from optparse import OptionParser

from threading import Thread
from SocketServer import ThreadingMixIn

#import epy_block_0
import pmt 
import pylab 
import limesdr
import thread
import time
import logging
import logging.handlers
import os 
import sys
import socket
import numpy
import numpy as np
import select
import sys

# Global declaration
bpskTxRxBlock      = 1        # Radio flow graph states
msgToSend          = ''       # Message to send
sdrSerNumber       = ''       # SDR serial number
radioTxPortNo      = ''       # Radio TX server port number
radioRxPortNo      = ''       # Radio RX server port number
argsSel            = 0        # Arguments selection
testMethod         = False
top_block_bpsk_cls = None     # Radio block reference class
top_block_bpsk     = None     # Radio block components
bpsk_tx_msg        = False    # Transmit message flag for method 2
limeSdrType        = False    # Default are limesdr mini usb
limeSdrChan        = False    # limesdr channel selection
lnaType            = 0        # RX LNA settings
paType             = 0        # TX PA settings
txBlckCentFreq     = 429.96e6 # Transmit frequency
dummyTxCentFreq    = 26.3e6
txBlckSmplRate     = 240e3 
dummyTxSmplRate    = 240e3
rxBlckCentFreq     = 429.96e6 # Receives frequency
dummyRxCentFreq    = 26.3e6
rxBlckSmplRate     = 240e3 
dummyRxSmplRate    = 240e3

# Get the arguments for SDR serial number
# Command example 01: python radio_bpsk_msg_test_v2.py 0009072C00D6331F 52002 52003 2 MINIUSB NA PAAUTO LNAAUTO
# Command example 02: python radio_bpsk_msg_test_v2.py 0009072C00D6331F 52002 52003 2 USB CHANA PAAUTO LNAAUTO

# Working pairing:
# limesdr mini USB: python radio_bpsk_msg_test_v2.py 1D4C42EEABAC8D 52000 52001 2 MINIUSB NA PAAUTO LNAAUTO -> TX[Band2] - RX[LNAW]
# limesdr USB: python radio_bpsk_msg_test_v2.py 0009072C00D6331F 52002 52003 2 USB CHANB PA1 LNAL -> TX[Band1] - RX[LNAL]
if (len(sys.argv) > 1):
    for x in sys.argv:
        if argsSel == 0 and x != 'radio_bpsk_msg_test_v2.py': 
            sdrSerNumber = x
            argsSel = 1
        elif argsSel == 1:
            radioTxPortNo = x
            argsSel = 2
        elif argsSel == 2:
            radioRxPortNo = x
            argsSel = 3
        elif argsSel == 3:
            if x == '1':
                testMethod = False
            elif x == '2':
                testMethod = True
                            
            argsSel = 4
        # Select limesdr type
        elif argsSel == 4:
            # Select limesdr USB
            if x == 'USB':
                limeSdrType = True
            # Select limesdr mini USB
            elif x == 'MINIUSB':
                limeSdrType = False
            argsSel = 5
        # Select limesdr channel
        elif argsSel == 5:
            # limesdr USB - Has 2 channel
            if limeSdrType == True:
                # Channel A
                if x == 'CHANA':
                    limeSdrChan = False
                elif x == 'CHANB':
                    limeSdrChan = True
            # limesdr mini USB - ONLY 1 channel
            elif limeSdrType == False:
                # Channel A
                if x == 'NA':
                    limeSdrChan = False
            argsSel = 6
        # TX PA settings
        elif argsSel == 6:
            # limesdr USB - Possible settings:
            # For channel A: Auto(default), band 1, band 2
            # For channel B: band 1, band 2
            if limeSdrType == True:
                # Channel A
                if limeSdrChan == False:
                    # Auto(default)
                    if x == 'PAAUTO':
                        paType = 0
                    # Band 1
                    elif x == 'PA1':
                        paType = 1
                    # Band 2
                    elif x == 'PA2':
                        paType = 2
                # Channel B
                elif limeSdrChan == True:
                    # Band 1
                    if x == 'PA1':
                        paType = 1
                    # Band 2
                    elif x == 'PA2':
                        paType = 2
            # limesdr mini USB - Possible settings - Auto(default), band 1, band 2
            elif limeSdrType == False:
                # Auto(default)
                if x == 'PAAUTO':
                    paType = 0
                # Band 1
                elif x == 'PA1':
                    paType = 1
                # Band 2
                elif x == 'PA2':
                    paType = 2
            argsSel = 7
        # RX LNA settings
        elif argsSel == 7:
            # limesdr USB - Possible settings:
            # For channel A: Auto(default), LNAH, LNAL, LNAW
            # For channel B: LNAH, LNAL, LNAW
            if limeSdrType == True:
                # Channel A
                if limeSdrChan == False:
                    # Auto(default)
                    if x == "LNAAUTO":
                        lnaType = 0
                    # LNAH
                    elif x == 'LNAH':
                        lnaType = 1
                    # LNAL
                    elif x == 'LNAL':
                        lnaType = 2
                    #LNAW
                    elif x == 'LNAW':
                        lnaType = 3
                # Channel B
                elif limeSdrChan == True:
                    # LNAH
                    if x == 'LNAH':
                        lnaType = 1
                    # LNAL
                    elif x == 'LNAL':
                        lnaType = 2
                    #LNAW
                    elif x == 'LNAW':
                        lnaType = 3
                
            # limesdr mini USB - Possible settings - Auto(default), LNAH, LNAL, LNAW
            elif limeSdrType == False:
                # Auto(default)
                if x == "LNAAUTO":
                    lnaType = 0
                # LNAH
                elif x == 'LNAH':
                    lnaType = 1
                # LNAL
                elif x == 'LNAL':
                    lnaType = 2
                #LNAW
                elif x == 'LNAW':
                    lnaType = 3
            break
                
# Debug parameters
if limeSdrType == False:
    print "DEBUG_PARAMS: limeSDR mini USB"
    
    if limeSdrChan == False:
        print "DEBUG_PARAMS: limeSDR mini USB - Channel A"
        
    if paType == 0:
        print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Auto(default)"
    elif paType == 1:
        print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 1"
    elif paType == 2:
        print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 2"

    if lnaType == 0:
        print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - Auto(default)"
    elif lnaType == 1:
        print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAH"
    elif lnaType == 2:
        print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAL"
    elif lnaType == 3:
        print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAW"
        
elif limeSdrType == True:
    print "DEBUG_PARAMS: limeSDR USB"
    if limeSdrChan == False:
        print "DEBUG_PARAMS: limeSDR USB - Channel A"

        if paType == 0:
            print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Auto(default)"
        elif paType == 1:
            print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 1"
        elif paType == 2:
            print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 2"

        if lnaType == 0:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - Auto(default)"
        elif lnaType == 1:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAH"
        elif lnaType == 2:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAL"
        elif lnaType == 3:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAW"
        
    elif limeSdrChan == True:
        print "DEBUG_PARAMS: limeSDR USB - Channel B"

        if paType == 1:
            print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 1"
        elif paType == 2:
            print "DEBUG_PARAMS: limeSDR mini USB - PA Path - Band 2"

        if lnaType == 1:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAH"
        elif lnaType == 2:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAL"
        elif lnaType == 3:
            print "DEBUG_PARAMS: limeSDR mini USB - LNA Path - LNAW"

#sys.exit()

# Class for BPSK messaging RX
class bpsk_msg_rx(gr.hier_block2):

    def __init__(self, my_const=digital.constellation_calcdist((digital.psk_2()[0]), (digital.psk_2()[1]), 2, 1).base()
):
        global sdrSerNumber
        global radioRxPortNo
        global limeSdrType
        global limeSdrChan        
        global lnaType
                           
        #gr.top_block.__init__(self, "BPSK RX Block")

        gr.hier_block2.__init__(self, "bpsk_msg_rx",
                                gr.io_signature(0, 0, 0), # Null signature
                                gr.io_signature(0, 0, 0))

        ##################################################
        # Parameters
        ##################################################
        self.my_const = my_const

        ##################################################
        # Variables
        ##################################################
        self.sps_TX = sps_TX = 40
        self.nfilts = nfilts = 32
        self.EBW = EBW = .35
        self.sps_RX = sps_RX = 40/10
        self.samp_rate = samp_rate = 240E3
        self.low_pass_filt_trans_width = low_pass_filt_trans_width = 1e6
        self.low_pass_filt_sampl_rate = low_pass_filt_sampl_rate = 2.048e6
        self.low_pass_filt_deci = low_pass_filt_deci = 1
        self.low_pass_filt_cut_off = low_pass_filt_cut_off = 100e3
        self.freq_offset_value = freq_offset_value = 30E3
        
        self.RRC_filter_taps = RRC_filter_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0, EBW, 5*sps_TX*nfilts)

        # Add variables for resetting
        self.rx_block_cent_freq = rx_block_cent_freq = 429.96e6
        self.rx_block_sampl_freq = rx_block_sampl_freq = 240e3

        ##################################################
        # Blocks
        ##################################################
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=int(sps_TX/sps_RX),
                taps=None,
                fractional_bw=None,
        )
        
##        #self.limesdr_source_0 = limesdr.source('1D4C42EEABAC8D', 0, '')
##        self.limesdr_source_0 = limesdr.source(sdrSerNumber, 0, '')
##        #self.limesdr_source_0.set_sample_rate(samp_rate)
##        self.limesdr_source_0.set_sample_rate(rx_block_sampl_freq)
##        #self.limesdr_source_0.set_center_freq(429.96e6, 0)
##        self.limesdr_source_0.set_center_freq(rx_block_cent_freq, 0)
##        self.limesdr_source_0.set_bandwidth(5e6,0)
##        self.limesdr_source_0.set_gain(60,0)
##        self.limesdr_source_0.set_antenna(255,0)
##        self.limesdr_source_0.calibrate(5e6, 0)

        self.limesdr_source_0 = None

        # Selection of the limesdr board
        # limesdr mini USB
        if limeSdrType == False:
            self.limesdr_source_0 = limesdr.source(sdrSerNumber, 0, '')
            self.limesdr_source_0.set_sample_rate(rx_block_sampl_freq)
            self.limesdr_source_0.set_center_freq(rx_block_cent_freq, 0)
            self.limesdr_source_0.set_bandwidth(5e6,0)
            self.limesdr_source_0.set_gain(60,0)

            # LNA - Auto (Default)
            if lnaType == 0:
                print "AUTO RX +++++++++++++++++++++++++++++++"
                self.limesdr_source_0.set_antenna(255,0)
            # LNAH
            elif lnaType == 1:
                self.limesdr_source_0.set_antenna(1,0)
            # LNAL
            elif lnaType == 2:
                self.limesdr_source_0.set_antenna(2,0)
            # LNAW
            elif lnaType == 3:
                self.limesdr_source_0.set_antenna(3,0)

            self.limesdr_source_0.calibrate(5e6, 0)

        # limesdr USB
        elif limeSdrType == True:
            # Channel A
            if limeSdrChan == False:
                self.limesdr_source_0 = limesdr.source(sdrSerNumber, 0, '')
                self.limesdr_source_0.set_sample_rate(rx_block_sampl_freq)
                self.limesdr_source_0.set_center_freq(rx_block_cent_freq, 0)
                self.limesdr_source_0.set_bandwidth(5e6,0)
                self.limesdr_source_0.set_gain(60,0)
            
                # LNA Path - Auto (Default)
                if lnaType == 0:
                    self.limesdr_source_0.set_antenna(255,0)
                # LNA Path - LNAH
                elif lnaType == 1:
                    self.limesdr_source_0.set_antenna(1,0)
                # LNA Path - LNAL
                elif lnaType == 2:
                    self.limesdr_source_0.set_antenna(2,0)
                # LNA Path - LNAW
                elif lnaType == 3:
                    self.limesdr_source_0.set_antenna(3,0)

                self.limesdr_source_0.calibrate(5e6, 0)

            # Channel B
            elif limeSdrChan == True:
                self.limesdr_source_0 = limesdr.source(sdrSerNumber, 1, '')
                self.limesdr_source_0.set_sample_rate(rx_block_sampl_freq)
                self.limesdr_source_0.set_center_freq(rx_block_cent_freq, 1)
                self.limesdr_source_0.set_bandwidth(5e6,1)
                self.limesdr_source_0.set_gain(60,1) 
                    
                # LNA Path - LNAH
                if lnaType == 1:
                    self.limesdr_source_0.set_antenna(1,1)
                # LNA Path - LNAL
                elif lnaType == 2:
                    self.limesdr_source_0.set_antenna(2,1)
                # LNA Path - LNAW
                elif lnaType == 3:
                    self.limesdr_source_0.set_antenna(3,1)

                self.limesdr_source_0.calibrate(5e6, 1)
            
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_ccf(sps_RX, 6.28/400.0*2, (RRC_filter_taps), nfilts, nfilts/2, 1.5, 1)
        self.digital_fll_band_edge_cc_0 = digital.fll_band_edge_cc(sps_RX, EBW, 45, .02)
        self.digital_diff_decoder_bb_0 = digital.diff_decoder_bb(2)
        self.digital_crc32_async_bb_0 = digital.crc32_async_bb(True)
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(.01, 2, False)
        self.digital_correlate_access_code_xx_ts_1_0 = digital.correlate_access_code_bb_ts(digital.packet_utils.default_access_code,
          2, 'len_key2')
        self.digital_constellation_soft_decoder_cf_0 = digital.constellation_soft_decoder_cf(my_const)
        self.digital_cma_equalizer_cc_0 = digital.cma_equalizer_cc(11, 1, .01, 1)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_tagged_stream_to_pdu_0 = blocks.tagged_stream_to_pdu(blocks.byte_t, 'len_key2')
        #self.blocks_socket_pdu_1 = blocks.socket_pdu("TCP_SERVER", '127.0.0.1', '52002', 10000, False)
        self.blocks_socket_pdu_1 = blocks.socket_pdu("TCP_SERVER", '127.0.0.1', radioRxPortNo, 10000, False)
        self.blocks_repack_bits_bb_0_0_0 = blocks.repack_bits_bb(1, 8, 'len_key2', False, gr.GR_MSB_FIRST)
        self.blocks_multiply_xx_1 = blocks.multiply_vcc(1)
        self.analog_sig_source_x_1 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, -freq_offset_value, 1, 0)
        self.analog_pwr_squelch_xx_0 = analog.pwr_squelch_cc(-20, .01, 0, True)
        self.analog_feedforward_agc_cc_0 = analog.feedforward_agc_cc(1024/2, 1.0)

        # Additional message debug block
        #self.blocks_message_debug_0 = blocks.message_debug()

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0, 'pdus'), (self.digital_crc32_async_bb_0, 'in'))    
        self.msg_connect((self.digital_crc32_async_bb_0, 'out'), (self.blocks_socket_pdu_1, 'pdus'))

        #self.msg_connect((self.digital_crc32_async_bb_0, 'out'), (self.blocks_message_debug_0, 'print'))
                
        self.connect((self.analog_feedforward_agc_cc_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))    
        self.connect((self.analog_pwr_squelch_xx_0, 0), (self.digital_fll_band_edge_cc_0, 0))    
        self.connect((self.analog_sig_source_x_1, 0), (self.blocks_multiply_xx_1, 1))    
        self.connect((self.blocks_multiply_xx_1, 0), (self.rational_resampler_xxx_0, 0))    
        self.connect((self.blocks_repack_bits_bb_0_0_0, 0), (self.blocks_tagged_stream_to_pdu_0, 0))    
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.digital_diff_decoder_bb_0, 0))    
        self.connect((self.digital_cma_equalizer_cc_0, 0), (self.digital_constellation_soft_decoder_cf_0, 0))    
        self.connect((self.digital_constellation_soft_decoder_cf_0, 0), (self.digital_binary_slicer_fb_0, 0))    
        self.connect((self.digital_correlate_access_code_xx_ts_1_0, 0), (self.blocks_repack_bits_bb_0_0_0, 0))    
        self.connect((self.digital_costas_loop_cc_0, 0), (self.digital_cma_equalizer_cc_0, 0))    
        self.connect((self.digital_diff_decoder_bb_0, 0), (self.digital_correlate_access_code_xx_ts_1_0, 0))    
        self.connect((self.digital_fll_band_edge_cc_0, 0), (self.analog_feedforward_agc_cc_0, 0))    
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.digital_costas_loop_cc_0, 0))    

        self.connect((self.limesdr_source_0, 0), (self.blocks_multiply_xx_1, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.analog_pwr_squelch_xx_0, 0))    

    def get_my_const(self):
        return self.my_const

    def set_my_const(self, my_const):
        self.my_const = my_const

    def get_sps_TX(self):
        return self.sps_TX

    def set_sps_TX(self, sps_TX):
        self.sps_TX = sps_TX

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_EBW(self):
        return self.EBW

    def set_EBW(self, EBW):
        self.EBW = EBW

    def get_sps_RX(self):
        return self.sps_RX

    def set_sps_RX(self, sps_RX):
        self.sps_RX = sps_RX

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_1.set_sampling_freq(self.samp_rate)

    # Setting back receives block center frequency
    def set_rx_block_cent_freq(self, rxfreq):
        self.rx_block_cent_freq = rxfreq
        self.limesdr_source_0.set_center_freq(self.rx_block_cent_freq, 0)

    # Setting back receives block sample frequency
    def set_rx_block_sampl_freq(self, samplrate):
        self.rx_block_sampl_freq = samplrate
        self.limesdr_source_0.set_sample_rate(self.rx_block_sampl_freq)

    def get_low_pass_filt_trans_width(self):
        return self.low_pass_filt_trans_width

    def set_low_pass_filt_trans_width(self, low_pass_filt_trans_width):
        self.low_pass_filt_trans_width = low_pass_filt_trans_width

    def get_low_pass_filt_sampl_rate(self):
        return self.low_pass_filt_sampl_rate

    def set_low_pass_filt_sampl_rate(self, low_pass_filt_sampl_rate):
        self.low_pass_filt_sampl_rate = low_pass_filt_sampl_rate

    def get_low_pass_filt_deci(self):
        return self.low_pass_filt_deci

    def set_low_pass_filt_deci(self, low_pass_filt_deci):
        self.low_pass_filt_deci = low_pass_filt_deci

    def get_low_pass_filt_cut_off(self):
        return self.low_pass_filt_cut_off

    def set_low_pass_filt_cut_off(self, low_pass_filt_cut_off):
        self.low_pass_filt_cut_off = low_pass_filt_cut_off

    def get_freq_offset_value(self):
        return self.freq_offset_value

    def set_freq_offset_value(self, freq_offset_value):
        self.freq_offset_value = freq_offset_value
        self.analog_sig_source_x_1.set_frequency(-self.freq_offset_value)

    def get_RRC_filter_taps(self):
        return self.RRC_filter_taps

    def set_RRC_filter_taps(self, RRC_filter_taps):
        self.RRC_filter_taps = RRC_filter_taps
        self.digital_pfb_clock_sync_xxx_0.update_taps((self.RRC_filter_taps))

# For testing purposes, not been used
# 
class msg_block_convert (gr.basic_block):
    # other base classes are basic_block, decim_block, interp_block """Convert strings to uint8 vectors""" 
    def __init__(self): 
        # only default arguments here """arguments to this function show up as parameters in GRC""" 
        gr.basic_block.__init__( self, name='msg_block_convert', # will show up in GRC 
                                in_sig=None, out_sig=None ) 
        self.message_port_register_out(pmt.intern('msg_out')) 
        self.message_port_register_in(pmt.intern('msg_in')) 
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_msg)
    
    def handle_msg(self, msg):
        # Cast the incoming msg to string
        #msg = str(msg)
        #msg = 'hello world'
        #nvec = pmt.to_python(msg)
        #print str(msg)
        nvec = str(msg)
        self.message_port_pub(pmt.intern('msg_out'), pmt.cons(pmt.make_dict(), pmt.pmt_to_python.numpy_to_uvector(np.array([ord(c) for c in nvec], np.uint8)))) 
    def work(self, input_items, output_items): 
            pass
    
# BPSK modulation packet detection - Detects complex value during TX digital modulation
class myBPSKtxdetector(gr.sync_block):
    # GMSK modulation TX progress detection embedded python block main entry script
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='myBPSKtxdetector',   # will show up in GRC
            in_sig=[np.complex64],
	    out_sig=None
        )
        self.cmplexVal = 0.0
        self.procCheck = False
        self.procEnd = False
        #self.gmskTxDetect

    # Get current BPSK modulation TX progress value
    def getCurrTxVal (self):
        # Return current GMSK modulation TX progress value
        return self.cmplexVal

    # Get current BPSK modulation text process status
    def getCurrTxtProc (self):
        # Return current BPSK modulation text process status
        return self.procEnd
    
    # Processing function
    def work(self, input_items, output_items):
        self.cmplexVal = input_items[0][0]
        self.cmplexVal = self.cmplexVal.real

        #print self.cmplexVal
        
        # Start check the text process
        if self.procCheck == False:
            # Text process in progress
            if self.cmplexVal > 0.0 or self.cmplexVal > -0.0:
                self.procCheck = True

        # Check the text process END
        else:
            # Text process end
            if self.cmplexVal == 0.0 or self.cmplexVal == -0.0:
                self.procCheck = False
                self.procEnd = True
                
        return len(input_items[0])

# Class for BPSK messaging TX
class bpsk_msg_tx(gr.hier_block2):

    def __init__(self, hdr_format=digital.header_format_default(digital.packet_utils.default_access_code, 0)
, my_const=digital.constellation_calcdist((digital.psk_2()[0]), (digital.psk_2()[1]), 2, 1).base()
):
        global sdrSerNumber
        global radioTxPortNo
        global dummyTxCentFreq
        global dummyTxSmplRate
        global limeSdrType
        global limeSdrChan        
        global paType
                
        #gr.top_block.__init__(self, "BPSK TX Block")

        gr.hier_block2.__init__(self, "bpsk_msg_tx",
                                gr.io_signature(0, 0, 0), # Null signature
                                gr.io_signature(0, 0, 0))

        ##################################################
        # Parameters
        ##################################################
        self.hdr_format = hdr_format
        self.my_const = my_const

        ##################################################
        # Variables
        ##################################################
        self.sps_TX = sps_TX = 40
        self.nfilts = nfilts = 32
        self.EBW = EBW = .35
        self.samp_rate = samp_rate = 240E3
        self.freq_offset_value = freq_offset_value = 30E3
        
        self.RRC_filter_taps = RRC_filter_taps = firdes.root_raised_cosine(nfilts, nfilts, 1.0, EBW, 5*sps_TX*nfilts)

        self.tx_block_cent_freq = tx_block_cent_freq = dummyTxCentFreq
        self.tx_block_sampl_freq = tx_block_sampl_freq = dummyTxSmplRate

        ##################################################
        # Blocks
        ##################################################
        self.pfb_arb_resampler_xxx_0 = pfb.arb_resampler_ccf(
        	  sps_TX,
                  taps=(RRC_filter_taps),
        	  flt_size=nfilts)
        self.pfb_arb_resampler_xxx_0.declare_sample_delay(0)

        ##################################################
        # Additional blocks for BPSK mosulation
        ##################################################
        # Block to detect complex value during text messaging process
        self.epy_block_0 = myBPSKtxdetector()
        #self.epy_block_1 = msg_block_convert()
        
##        #self.limesdr_sink_0_0 = limesdr.sink('0009072C00D6331F', 0, '', '')
##        self.limesdr_sink_0_0 = limesdr.sink(sdrSerNumber, 0, '', '')
##        #self.limesdr_sink_0_0.set_sample_rate(samp_rate)
##        self.limesdr_sink_0_0.set_sample_rate(tx_block_sampl_freq)
##        #self.limesdr_sink_0_0.set_center_freq(429.96e6, 0)
##        self.limesdr_sink_0_0.set_center_freq(tx_block_cent_freq, 0)
##        self.limesdr_sink_0_0.set_bandwidth(5e6,0)
##        self.limesdr_sink_0_0.set_gain(60,0)
##        self.limesdr_sink_0_0.set_antenna(255,0)
##        self.limesdr_sink_0_0.calibrate(5e6, 0)

        self.limesdr_sink_0_0 = None
        
        # Selection of the limesdr board
        # limesdr mini USB
        if limeSdrType == False:
            print tx_block_sampl_freq
            
            self.limesdr_sink_0_0 = limesdr.sink(sdrSerNumber, 0, '', '')
            self.limesdr_sink_0_0.set_sample_rate(tx_block_sampl_freq)
            self.limesdr_sink_0_0.set_center_freq(tx_block_cent_freq, 0)
            self.limesdr_sink_0_0.set_bandwidth(5e6,0)
            self.limesdr_sink_0_0.set_gain(60,0)

            # PA Path - Auto (Default)
            if paType == 0:
                print "AUTO TX +++++++++++++++++++++++++++++++"
                self.limesdr_sink_0_0.set_antenna(255,0)
            # PA Path - Band 1
            elif paType == 1:
                self.limesdr_sink_0_0.set_antenna(1,0)
            # PA Path - Band 2
            elif paType == 2:
                self.limesdr_sink_0_0.set_antenna(2,0)
            
            self.limesdr_sink_0_0.calibrate(5e6, 0)

        # limesdr USB
        elif limeSdrType == True:
            # Channel A
            if limeSdrChan == False:
                self.limesdr_sink_0_0 = limesdr.sink(sdrSerNumber, 0, '', '')
                self.limesdr_sink_0_0.set_sample_rate(tx_block_sampl_freq)
                self.limesdr_sink_0_0.set_center_freq(tx_block_cent_freq, 0)
                self.limesdr_sink_0_0.set_bandwidth(5e6,0)
                self.limesdr_sink_0_0.set_gain(60,0)

                # PA Path - Auto (Default)
                if paType == 0:
                    self.limesdr_sink_0_0.set_antenna(255,0)
                # PA Path - Band 1
                elif paType == 1:
                    self.limesdr_sink_0_0.set_antenna(1,0)
                # PA Path - Band 2
                elif paType == 2:
                    self.limesdr_sink_0_0.set_antenna(2,0)
                
                self.limesdr_sink_0_0.calibrate(5e6, 0)

            # Channel B
            elif limeSdrChan == True:
                self.limesdr_sink_0_0 = limesdr.sink(sdrSerNumber, 1, '', '')
                self.limesdr_sink_0_0.set_sample_rate(tx_block_sampl_freq)
                self.limesdr_sink_0_0.set_center_freq(tx_block_cent_freq, 0)
                self.limesdr_sink_0_0.set_bandwidth(5e6,1)
                self.limesdr_sink_0_0.set_gain(60,1)

                # PA Path - Band 1
                if paType == 1:
                    self.limesdr_sink_0_0.set_antenna(1,1)
                # PA Path - Band 2
                elif paType == 2:
                    self.limesdr_sink_0_0.set_antenna(2,1)
                
                self.limesdr_sink_0_0.calibrate(5e6, 1)
            
        self.digital_protocol_formatter_bb_0 = digital.protocol_formatter_bb(hdr_format, 'len_key')
        self.digital_diff_encoder_bb_0 = digital.diff_encoder_bb(2)
        self.digital_crc32_async_bb_1 = digital.crc32_async_bb(False)
        self.digital_chunks_to_symbols_xx_0_0 = digital.chunks_to_symbols_bc((my_const.points()), 1)
        self.digital_burst_shaper_xx_0 = digital.burst_shaper_cc((numpy.ones(500)), 4000, 4000, True, 'len_key')
        (self.digital_burst_shaper_xx_0).set_block_alias("burst_shaper0")
        self.blocks_tagged_stream_mux_0 = blocks.tagged_stream_mux(gr.sizeof_char*1, 'len_key', 0)
        self.blocks_tagged_stream_multiply_length_0 = blocks.tagged_stream_multiply_length(gr.sizeof_gr_complex*1, 'len_key', sps_TX)
        #self.blocks_socket_pdu_0 = blocks.socket_pdu("TCP_SERVER", '127.0.0.1', '52001', 10000, False)
        self.blocks_socket_pdu_0 = blocks.socket_pdu("TCP_SERVER", '127.0.0.1', radioTxPortNo, 10000, False)
        self.blocks_repack_bits_bb_0_0 = blocks.repack_bits_bb(8, my_const.bits_per_symbol(), 'len_key', False, gr.GR_MSB_FIRST)
        self.blocks_pdu_to_tagged_stream_1 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'len_key')
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((0.5, ))
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, freq_offset_value, 1, 0)

        ##################################################
        # Connections
        ##################################################
        ##
        #self.msg_connect((self.blocks_socket_pdu_0, 'pdus'), (self.epy_block_1, 'msg_in'))
        #self.msg_connect((self.epy_block_1, 'msg_out'), (self.digital_crc32_async_bb_1, 'in'))
        ##
        
        self.msg_connect((self.blocks_socket_pdu_0, 'pdus'), (self.digital_crc32_async_bb_1, 'in'))    
        self.msg_connect((self.digital_crc32_async_bb_1, 'out'), (self.blocks_pdu_to_tagged_stream_1, 'pdus'))    
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 1))    
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_multiply_xx_0, 0))    

        self.connect((self.blocks_multiply_xx_0, 0), (self.limesdr_sink_0_0, 0))

        # Additional connections
        self.connect((self.blocks_multiply_xx_0, 0), (self.epy_block_0, 0))
                        
        self.connect((self.blocks_pdu_to_tagged_stream_1, 0), (self.blocks_tagged_stream_mux_0, 1))    
        self.connect((self.blocks_pdu_to_tagged_stream_1, 0), (self.digital_protocol_formatter_bb_0, 0))    
        self.connect((self.blocks_repack_bits_bb_0_0, 0), (self.digital_diff_encoder_bb_0, 0))    
        self.connect((self.blocks_tagged_stream_multiply_length_0, 0), (self.blocks_multiply_const_vxx_0, 0))    
        self.connect((self.blocks_tagged_stream_mux_0, 0), (self.blocks_repack_bits_bb_0_0, 0))    
        self.connect((self.digital_burst_shaper_xx_0, 0), (self.pfb_arb_resampler_xxx_0, 0))    
        self.connect((self.digital_chunks_to_symbols_xx_0_0, 0), (self.digital_burst_shaper_xx_0, 0))    
        self.connect((self.digital_diff_encoder_bb_0, 0), (self.digital_chunks_to_symbols_xx_0_0, 0))    
        self.connect((self.digital_protocol_formatter_bb_0, 0), (self.blocks_tagged_stream_mux_0, 0))    
        self.connect((self.pfb_arb_resampler_xxx_0, 0), (self.blocks_tagged_stream_multiply_length_0, 0))

    # Get text messaging status during START and END of transmisson
    def get_bpsktxstatus(self):
        return self.epy_block_0.getCurrTxtProc()

    def get_hdr_format(self):
        return self.hdr_format

    def set_hdr_format(self, hdr_format):
        self.hdr_format = hdr_format

    def get_my_const(self):
        return self.my_const

    def set_my_const(self, my_const):
        self.my_const = my_const

    def get_sps_TX(self):
        return self.sps_TX

    def set_sps_TX(self, sps_TX):
        self.sps_TX = sps_TX
        self.pfb_arb_resampler_xxx_0.set_rate(self.sps_TX)
        self.blocks_tagged_stream_multiply_length_0.set_scalar(self.sps_TX)

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts

    def get_EBW(self):
        return self.EBW

    def set_EBW(self, EBW):
        self.EBW = EBW

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)

    # Setting back transmit block center frequency
    def set_tx_block_cent_freq(self, txfreq):
        self.tx_block_cent_freq = txfreq
        self.limesdr_sink_0_0.set_center_freq(self.tx_block_cent_freq, 0)

    # Setting back transmit block sample frequency
    def set_tx_block_sampl_freq(self, samplrate):
        self.tx_block_sampl_freq = samplrate
        self.limesdr_sink_0_0.set_sample_rate(self.tx_block_sampl_freq)

    def get_freq_offset_value(self):
        return self.freq_offset_value

    def set_freq_offset_value(self, freq_offset_value):
        self.freq_offset_value = freq_offset_value
        self.analog_sig_source_x_0.set_frequency(self.freq_offset_value)

    def get_RRC_filter_taps(self):
        return self.RRC_filter_taps

    def set_RRC_filter_taps(self, RRC_filter_taps):
        self.RRC_filter_taps = RRC_filter_taps
        self.pfb_arb_resampler_xxx_0.set_taps((self.RRC_filter_taps))

# GNU radio class for BPSK
class BPSK_Transceiver (gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self, "BPSK_Transceiver")

        self.bpsk_rx_path = bpsk_msg_rx()
        self.bpsk_tx_path = bpsk_msg_tx()
        
        self.connect(self.bpsk_rx_path)
        self.connect(self.bpsk_tx_path)
        
    # Set transmit block center frequency
    def set_BPSK_transmit_centfreq(self, centfreq):
        self.bpsk_tx_path.set_tx_block_cent_freq(centfreq)

    # Set transmit block sample rate
    def set_BPSK_transmit_samplerate(self, samplrate):
        self.bpsk_tx_path.set_tx_block_sampl_freq(samplrate)

    # Get transmission status
    def get_msg_tx_status(self):
        retStatus = False
        retStatus = self.bpsk_tx_path.get_bpsktxstatus()
        return retStatus

    # Set receives block center frequency
    def set_BPSK_receive_centfreq(self, centfreq):
        self.bpsk_rx_path.set_rx_block_cent_freq(centfreq)

    # Set receives block sample rate
    def set_BPSK_receive_samplerate(self, samplrate):
        self.bpsk_rx_path.set_rx_block_sampl_freq(samplrate)

# Thread to send message via BPSK modulation 
def threadTXMsg (threadname, delay):
    global radioTxPortNo
    global bpsk_tx_msg
    global msgToSend
    global top_block_bpsk
    global dummyTxCentFreq
    global dummyTxSmplRate
    global rxBlckCentFreq
    global rxBlckSmplRate
            
    sockConnected = False # Socket connection flag
    sockSendMsg = False
    resendMsg = False
    servSockObj = None
    dlyRstTx = False
    dlyRstTxCnt = 0
    sendCnt = 0
    #dummyTx = '7C407F7F773072642A21'
    dummyTx = 'xxxx'

    # Connection attempt to the BPSK radio server (TX) for a first time
    try:
        servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        servSockObj.connect(('127.0.0.1', int(radioTxPortNo)))

        print "THD_BPSK_TX: Connected to BPSK radio server (TX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

        # Connection success
        sockConnected = True
    
    # Connection failed
    except:
         print "THD_BPSK_TX: First connection attempt to BPSK radio server (TX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

    # Thread main loop
    while(True):
        time.sleep(delay)
        # Try to make a connection to the BPSK radio server (TX)
        try:
            # Connection attempt to server
            if sockConnected == False:
                servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                servSockObj.connect(('127.0.0.1', int(radioTxPortNo)))

                print "THD_BPSK_TX: Connected to BPSK radio server (TX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

                # Connection success
                sockConnected = True

            # Previously has already connected to the BPSK radio server (TX)
            else:
                # Previously there is a message that need to be sends
                if bpsk_tx_msg == True:
                    # Send text message to BPSK radio server (TX)
                    if sockSendMsg == False:
                        # Send start transmission dummy contents first
                        if sendCnt == 0:
                            servSockObj.send(dummyTx)
                        # Then send the actual message contents
                        else:
                            servSockObj.send(msgToSend)
                        
                        # Increment counter before sending actual message contents
                        sendCnt += 1

                        print "THD_BPSK_TX: Start send message to BPSK radio server, waiting for TX finished status"
                        sockSendMsg = True

                    # Check the ends of text message transmission
                    else:
                        # Check text message transmission status, either already finished or not
                        txtProcStat = top_block_bpsk.get_msg_tx_status()
                        # Text message transmission has finished
                        if txtProcStat == True:
                            # Complete message transmission
                            if sendCnt == 2:
                                sendCnt = 0
                                
                                print "THD_BPSK_TX: Message transmission via radio link ends"

                                sockSendMsg = False
                                dlyRstTx = True

                                bpsk_tx_msg = False
                            # Prepare to send actual message contents on the next loop
                            else:
                                sockSendMsg = False

                # Delay 5s before start resetting transmit and receive frequency
                elif dlyRstTx == True:
                    dlyRstTxCnt += 1
                    # Reach 5s start resetting frequency
                    if dlyRstTxCnt == 50:
                        dlyRstTxCnt = 0

                        # Resetting back transmit frequency and sample rate, STOP transmitting
                        top_block_bpsk.set_BPSK_transmit_centfreq(dummyTxCentFreq)
                        top_block_bpsk.set_BPSK_transmit_samplerate(dummyTxSmplRate)

                        # Resetting back receive frequency and sample rate, START receives messages
                        top_block_bpsk.set_BPSK_receive_centfreq(rxBlckCentFreq)
                        top_block_bpsk.set_BPSK_receive_samplerate(rxBlckSmplRate)

                        print "THD_BPSK_TX: RESET TX frequency"

                        msgToSend = ''
                        dlyRstTx = False
                 
        # Connection retry failed
        except:
            print "THD_BPSK_TX: Connection attempt to BPSK radio server (TX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)  

# Thread to receives message via BPSK modulation
# Add a protocol 
def threadRXMsg (threadname, delay):
    global radioRxPortNo
    global bpsk_tx_msg
    global msgToSend
    global top_block_bpsk

    global txBlckCentFreq
    global txBlckSmplRate
    
    global dummyRxCentFreq
    global dummyRxSmplRate

    sockConnected = False # Socket connection flag
    servSockObj = None
    sockets_list = []     # Sockets list
    read_sockets = []
    write_socket = []
    error_socket = []

    # Connection attempt to the BPSK radio server (RX) for a first time
    try:
        servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        servSockObj.connect(('127.0.0.1', int(radioRxPortNo)))

        print "THD_BPSK_RX: Connected to BPSK radio server (RX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

        # maintains a list of possible input streams
        sockets_list = [sys.stdin, servSockObj]
    
        # Connection success
        sockConnected = True

    # Connection failed
    except:
         print "THD_BPSK_RX: First connection attempt to BPSK radio server (RX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)
             
    # Thread main loop
    while(True):
        time.sleep(delay)
        # Try to make a connection to the BPSK radio server (RX)
        try:
            # Connection attempt to server
            if sockConnected == False:
                servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                servSockObj.connect(('127.0.0.1', int(radioRxPortNo)))
        
                print "THD_BPSK_RX: Connected to BPSK radio server (RX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

                # maintains a list of possible input streams
                sockets_list = [sys.stdin, servSockObj]
        
                # Connection success
                sockConnected = True

            # Previously has already connected to the BPSK radio server (RX)
            else:
                read_sockets,write_socket, error_socket = select.select(sockets_list,[],[])
                # Go through the read sockets list
                for socks in read_sockets:
                    # There is a data from BPSK radio server (RX)
                    if socks == servSockObj:
                        # Initialize receive data variable
                        rxMsgData = servSockObj.recv(4096)

                        # Don't display the dummy txt message
                        #if rxMsgData != '7C407F7F773072642A21':
                        if rxMsgData != 'xxxx':
                            print "####################################################################"
                            print " "
                            print "BPSK_RX: Receive BPSK message: %s" % (rxMsgData)
                            print " "
                            print "####################################################################"
                            
                        # Connection with server closed
                        if rxMsgData == b'':
                            print "THD_BPSK_RX: Connection with BPSK radio server (RX) CLOSED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo) 

                            servSockObj.close()
                            sockConnected = False
                            rxMsgData = ''
                            
                            break

                        rxMsgData = ''
                        
                    # No data or sockets are idle, waiting for input message to send
                    else:
                        #print "THD_BPSK_RX: Please type the message to send and press enter...."
                        # Waiting for input to sends a message
                        msgToSend = sys.stdin.readline()
                        #dispMsg = 'BPSK_RX: Receives BPSK message: ' + msgToSend

                        print "####################################################################"
                        print " "
                        print "BPSK_TX: Transmit BPSK message: %s" % (msgToSend)
                        print " "
                        print "####################################################################"
                            
                        #sys.stdout.write("####################################################################\n")
                        #sys.stdout.write(" \n")
                        #sys.stdout.write(dispMsg)
                        #sys.stdout.write(" ")
                        #sys.stdout.write("####################################################################")
                        #sys.stdout.flush()

                        # Resetting back receive frequency and sample rate, STOP receives anything
                        top_block_bpsk.set_BPSK_receive_centfreq(dummyRxCentFreq)
                        top_block_bpsk.set_BPSK_receive_samplerate(dummyRxSmplRate)
                        
                        # Resetting back transmit frequency and sample rate, actual transmit frequency
                        top_block_bpsk.set_BPSK_transmit_centfreq(txBlckCentFreq)
                        top_block_bpsk.set_BPSK_transmit_samplerate(txBlckSmplRate)

                        # Set transmit flag
                        bpsk_tx_msg = True

        # Connection retry failed   
        except:
            print "THD_BPSK_RX: Connection attempt to BPSK radio server (RX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)
        
# Thread to receive and transmit messages via radio class
# Connect to the PDU server inside BPSK receives message class
def threadRxTxMsg (threadname, delay):
    global bpskTxRxBlock
    global msgToSend
    global bpskTxRxBlock
    global radioTxPortNo                                              
    global radioRxPortNo

    sockConnected = False # Socket connection flag
    sendMsgStat = 0
    rxMsgData = ''        # Received message data from radio
    sockets_list = []     # Sockets list
    read_sockets = []
    write_socket = []
    error_socket = []

    # BPSK radio server (RX) process are start here, inside thread
    # With this method BPSK radio server (RX) block can be terminated instantly
    top_block_cls = None  # Radio block reference class
    top_block = None      # Radio block components

    # BPSK radio server in RX mode 
    if bpskTxRxBlock == 1:
        # Initializing block in BPSK modulation 
        top_block_cls = bpsk_msg_rx
        top_block = top_block_cls()
            
        # Start the BPSK modulation block
        top_block.start()

        # Connection attempt to the BPSK radio server (RX) for a first time
        try:
            servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            servSockObj.connect(('127.0.0.1', int(radioRxPortNo)))

            print "THD_BPSK_TX_RX: Connected to BPSK radio server (RX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

            # maintains a list of possible input streams
            sockets_list = [sys.stdin, servSockObj]
        
            # Connection success
            sockConnected = True

        # Connection failed
        except:
             print "THD_BPSK_TX_RX: First connection attempt to BPSK radio server (RX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

    # BPSK radio server in TX mode
    elif bpskTxRxBlock == 2:
        # Initializing block in BPSK modulation 
        top_block_cls = bpsk_msg_tx
        top_block = top_block_cls()
            
        # Start the BPSK modulation block
        top_block.start()
        
        # Connection attempt to the BPSK radio server (TX) for a first time
        try:
            servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            servSockObj.connect(('127.0.0.1', int(radioTxPortNo)))

            print "THD_BPSK_TX_RX: Connected to BPSK radio server (TX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

            # Connection success
            sockConnected = True

        # Connection failed
        except:
             print "THD_BPSK_TX_RX: First connection attempt to BPSK radio server (TX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

    # Thread main loop
    while(True):
        time.sleep(delay)
        # BPSK radio server in RX mode 
        if bpskTxRxBlock == 1:
            # Try to make a connection to the BPSK radio server (TX or RX)
            try:
                # Connection attempt to server
                if sockConnected == False:
                    servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    servSockObj.connect(('127.0.0.1', int(radioRxPortNo)))
            
                    print "THD_BPSK_TX_RX: Connected to BPSK radio server (RX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

                    # maintains a list of possible input streams
                    sockets_list = [sys.stdin, servSockObj]
            
                    # Connection success
                    sockConnected = True

                # Previously has already connected to the BPSK radio server (RX)
                else:
                    read_sockets,write_socket, error_socket = select.select(sockets_list,[],[])
                    # Go through the read sockets list
                    for socks in read_sockets:
                        # There is a data from BPSK radio server (RX)
                        if socks == servSockObj:
                            # Initialize receive data variable
                            rxMsgData = ''
                            rxVidStatData = servSockObj.recv(4096)

                            print "THD_BPSK_TX_RX: Receives BPSK message: %s" % (rxVidStatData)
                            
                            # Connection with server closed
                            if rxVidStatData == b'':
                                print "THD_BPSK_TX_RX: Connection with BPSK radio server (RX) CLOSED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

                                servSockObj.close()
                                sockConnected = False

                                break;

                        # No data or sockets are idle, waiting for input message to send
                        else:
                            print "THD_BPSK_TX_RX: Please type the message to send and press enter...."
                            # Waiting for input to sends a message
                            msgToSend = sys.stdin.readline()
                            # Initiate back thread for BPSK radio server (TX) from the main loop
                            bpskTxRxBlock = 3
                            # Exit read sockets list
                            break

                    # Prepare to exit thread and stop the BPSK radio server (RX)
                    if bpskTxRxBlock == 3:
                        print "THD_BPSK_TX_RX: EXIT BPSK radio server (RX) thread, ready to initiate radio in TX mode"
                        
                        # STOP BPSK radio server (RX)
                        top_block.stop()
                        top_block.wait()

                        servSockObj.close()
                        break
                        
            # Connection retry failed   
            except:
                print "THD_BPSK_TX_RX: Connection attempt to BPSK radio server (RX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioRxPortNo)

        # BPSK radio server in TX mode
        elif bpskTxRxBlock == 2:
            # Try to make a connection to the BPSK radio server (TX)
            try:
                # Connection attempt to server
                if sockConnected == False:
                    servSockObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    servSockObj.connect(('127.0.0.1', int(radioTxPortNo)))

                    print "THD_BPSK_TX_RX: Connected to BPSK radio server (TX) SUCCESSFUL: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)

                    # Connection success
                    sockConnected = True

                # Previously has already connected to the BPSK radio server (TX)
                else:
                    # Send message to BPSK radio server (TX)
                    if sendMsgStat == 0:
                        #print msgToSend
                        servSockObj.send(msgToSend)

                        print "THD_BPSK_TX_RX: Start send message to BPSK radio server, waiting for TX finished status"
                        
                        sendMsgStat = 1

                    # Observed the end of message transmission
                    elif sendMsgStat == 1:
                        # Check text message transmission status, either already finished or not
                        txtProcStat = top_block.get_bpsktxstatus()
                        # Text message transmission has finished
                        if txtProcStat == True:
                            print "THD_BPSK_TX_RX: Message transmission via radio link ends"
                            print "THD_BPSK_TX_RX: EXIT BPSK radio server (TX) thread, ready to initiate radio in RX mode"

                            # STOP BPSK radio server (TX)
                            top_block.stop()
                            top_block.wait()

                            servSockObj.close()

                            # Initiate back thread for BPSK radio server (RX) from the main loop
                            bpskTxRxBlock = 4
                            sendMsgStat = 0
                            break
                            
            # Connection retry failed
            except:
                print "THD_BPSK_TX_RX: Connection attempt to BPSK radio server (TX) FAILED!: IP: 127.0.0.1 Port No.: %s" % (radioTxPortNo)  
    
# Script entry point
def main():
    global bpskTxRxBlock
    global testMethod
    global top_block_bpsk_cls
    global top_block_bpsk

    # Method 1
    if testMethod == False:
        try:
            thread.start_new_thread(threadRxTxMsg, ("threadRxTxMsg", 0.1))
        except:
            print "THREAD_ERROR: Unable to start [threadRxTxMsg] thread"

    # Method 2
    else:
        # Initialize BPSK transceiver class
        top_block_bpsk_cls = BPSK_Transceiver
        top_block_bpsk = top_block_bpsk_cls()

        # Start the BPSK transceiver
        top_block_bpsk.start()

        # Start RX thread
        try:
            thread.start_new_thread(threadRXMsg, ("threadRXMsg", 0.1))
        except:
            print "THREAD_ERROR: Unable to start [threadRXMsg] thread"

        # Start TX thread
        try:
            thread.start_new_thread(threadTXMsg, ("threadTXMsg", 0.1))
        except:
            print "THREAD_ERROR: Unable to start [threadTXMsg] thread"
            
    # Main loop
    while True:
        time.sleep(1)

        # Method 1
        if testMethod == False:
            # Restart radio flow graph in BPSK radio (TX)
            if bpskTxRxBlock == 3:
                bpskTxRxBlock = 2

                try:
                    thread.start_new_thread(threadRxTxMsg, ("threadRxTxMsg", 0.5))
                except:
                    print "THREAD_ERROR: Unable to start [threadRxTxMsg] thread"

            # Restart radio flow graph in BPSK radio (RX)
            elif bpskTxRxBlock == 4:
                bpskTxRxBlock = 1

                try:
                    thread.start_new_thread(threadRxTxMsg, ("threadRxTxMsg", 0.5))
                except:
                    print "THREAD_ERROR: Unable to start [threadRxTxMsg] thread"
    
if __name__ == '__main__':
    main()    
    
