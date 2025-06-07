# 	This code has been developed for the Raspberry Pi Pico W connected to a Pimoroni 16x7 LED matrix.
#	
#	Uses example from the documentation as a basis for DMA chaining:
# 		https://docs.micropython.org/en/latest/library/rp2.DMA.html
# 	Uses the pio from here (changed to pythonic version and slightly modified):
#       https://github.com/pimoroni/pimoroni-pico/blob/main/libraries/pico_unicorn/pico_unicorn.pio


from rp2 import DMA
from uctypes import addressof
from array import array
import time
import pio
from machine import Pin
import time
from machine import mem32
from machine import Timer

TXF0_addr = const(0x50200010)   # address of TX FIFO register for State Machine 0
RXF0_addr = const(0x50200020)   # address of RX FIFO register for State Machine 0
UPDATE_IMAGE_FLAG = 0


def transfer_data(input_array):
    """
        transfer_data transfers the input array to the PIO TX FIFO register. 
        
        This method uses DMA chaining:
         - gather_dma chains to the buffer_dma,
         - buffer_dma chains to reset_write_dma,
         - reset_write_dma chains to reset_read_dma,
         - reset_read_dma chains to itself, meaning that chaining is disabled for this channel.
         
        gather_dma, as per the example in the micropython documentation (see link above), directly populates
        the buffer_dma registers with:
         - the number of transfers (COUNT register)
         - the address of the start of the input_array array.
        (Please note that for every reconfiguration that gather_dma gives to buffer_dma, it points it at an
        array containing a full frame. This means that buffer_dma does not need to chain back to gather_dma,
        and we can chain from it to another dma to reset the gather dma.)
        
        Then, buffer_dma is triggered/turned to active by gather_dma once gather_dma writes to buffer_dma
        registers, and buffer_dma writes data to the PIO TX FIFO.
        
        Once buffer_dma completes its *full frame transfer*, it chains to reset_write_dma. This resets the
        gather_dma write address ONLY, and doesnt retrigger it.
        
        Once reset_write_dma completes its single transfer to the write register of gather_dma, it chains to
        the reset_read_dma, which then writes to the read register of the gather_dma (which also doubles up as
        a trigger - this is specifically the CHx_AL3_READ_ADDR_TRIG). The reason for having two DMAs complete
        a two word transfers to a single dma config, rather than just using one, is because we want to set the
        inc_read and incr_write to False - otherwise we would need to have something separate to reset the reset
        DMA... but this could actually be a better solution if we want to transfer more than two config changes
        to a single DMA.
        
        This means that this can now run in the background with interrupt laden libraries, such as the network
        library, and no matrix flicker will occur. 
        

    """
    #------------------------------------- STATE MACHINE 0 DEFINITION -----------------------------------#
    # define the state machine first
    pin_sin = 8
    pin_sclk = 9
    pin_sr6 = 16
    rp2.PIO(0).remove_program() # this is only used as a precaution 
    sm0 = rp2.StateMachine(0, pio.shift_reg, freq=8_000_000, sideset_base=Pin(pin_sclk), set_base=Pin(pin_sin), out_base=Pin(pin_sr6), out_shiftdir=rp2.PIO.SHIFT_RIGHT)
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- DMA INIT ---------------------------------------------#
    # We use two DMA channels. The first sends lengths and source addresses from the gather
    # list to the registers of the second. The second copies the data itself.
    gather_dma = DMA()
    buffer_dma = DMA()
    reset_write_dma = DMA()
    reset_read_dma = DMA()
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER LIST ------------------------------------------#
    # Pack up length/address pair of input_array to be sent to the buffer_dma registers.
    # This only works for a single input_array = array("L", [])
    gather_list = array("L")
    
    gather_list.append(len(input_array))
    gather_list.append(addressof(input_array))
    
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER DMA -------------------------------------------#
    # From the example: 
    # "
    #     When writing to the registers of the second DMA channel, we need to wrap the
    #     write address on an 8-byte (1<<3 bytes) boundary. We write to the ``TRANS_COUNT``
    #     and ``READ_ADD_TRIG`` registers in the last register alias (registers 14 and 15
    #     respectively).
    # "
    # Just a side note in case I forget - if we want to write to addresses 13, 14 and 15, we would to the
    # following:
    #  - make sure gather list has all the desired register contents in the right order.
    #  - ring_size = 4
    #  - count = 3
    #  - change from write=buffer_dma.registers[14:16] to write=buffer_dma.registers[13:16]
    # But we only want to do what the example suggests.
    
    gather_ctrl = gather_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=True,
        inc_write=True,
        ring_size=3,
        ring_sel=True,
        treq_sel=63,
        irq_quiet=True,
        bswap=False,
        sniff_en=False,
        chain_to=buffer_dma.channel
    )   
    
    gather_dma.config(
        read=addressof(gather_list), write=buffer_dma.registers[14:16],
        count=2, ctrl=gather_ctrl, trigger=False
    )
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- BUFFER DMA -------------------------------------------#
   
    # for sending data to the pio.
    buffer_ctrl = buffer_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=True,   
        inc_write=False,  
        ring_size=0,
        ring_sel=False,
        treq_sel=0,       
        irq_quiet=True,
        bswap=False,
        sniff_en=False,
        chain_to=reset_write_dma.channel 
    )
        
        
    # The read and count values will be set by the other DMA channel.
    buffer_dma.config(write=TXF0_addr, ctrl=buffer_ctrl, trigger=False)

    #----------------------------------------------------------------------------------------------------#
        
    #--------------------------------------------- RESET DMAS --------------------------------------------#
    # for our reset DMA, we really do not want to be increasing read or the write. This is because it would
    # then require our reset DMA to be reset... so we need to chain two DMAs together, one that first sets
    # the write address register of gather_dma, then one that sets the read address register (which doubles
    # up as a trigger too.
       
    reset_write_ctrl = reset_write_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=False,
        inc_write=False,
        ring_size=0,
        ring_sel=False,
        treq_sel=63,
        irq_quiet=True,
        bswap=False,
        sniff_en=False,
        chain_to=reset_read_dma.channel
    )

    
    # want to reset the address to which gather DMA write to, which is buffer_dma.registers[14:16]
    # so we want to transfer the address of the registers rather than the registers themselves. 
    buffer_dma_write = array("L", [ addressof(buffer_dma.registers[14:14]) ])
    
    reset_write_dma.config(read=addressof(buffer_dma_write), write=gather_dma.registers[13:13], count=1, ctrl=reset_write_ctrl, trigger=False)
    
    reset_read_ctrl = reset_read_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=False,
        inc_write=False,
        ring_size=0,
        ring_sel=False,
        treq_sel=63,
        irq_quiet=True,
        bswap=False,
        sniff_en=False,
        chain_to=reset_read_dma.channel
    )
    
    gather_list_addr = array("L", [addressof(gather_list)])
    
    reset_read_dma.config(read=addressof(gather_list_addr), write=gather_dma.registers[15:15], count=1, ctrl=reset_read_ctrl, trigger=False)

    #----------------------------------------------------------------------------------------------------#
        
    #--------------------------------------------- ACTIVATION -------------------------------------------#    
    
    # activate state machine
    sm0.active(1)
    
    # activate gather_dma. Do not activate buffer_dma, as gather_dma will trigger it once it properly sets
    # buffer_dma up.
    gather_dma.active(1)
    
    print("DMAs commenced.")
    
    return gather_dma, buffer_dma, reset_write_dma, reset_read_dma, sm0

    #----------------------------------------------------------------------------------------------------#

def update_image_flag_set(t):
    global UPDATE_IMAGE_FLAG
    UPDATE_IMAGE_FLAG = 1
    
#### MAIN ####
input = array("L", [0, 0,  0,  0, 0,  0, 0,  0, 253, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 251, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 247, 0, 4095,
                        0, 34, 34, 34,34, 0, 34, 0, 239, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 223, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 191, 0, 4095,
                        0, 0,  0,  0, 0,  0, 0,  0, 127, 0, 4095])

input_red = array("L", [0, 0,  0,  0, 0,  0, 0,  0, 253, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 251, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 247, 0, 4095,
                        0, 34, 34, 34,34, 0, 34, 0, 239, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 223, 0, 4095,
                        0, 34, 0,  0, 34, 0, 34, 0, 191, 0, 4095,
                        0, 0,  0,  0, 0,  0, 0,  0, 127, 0, 4095])

input_blue = array("L", [0, 0,  0,  0, 0,  0, 0,  0, 253, 0, 4095,
                        0, 68, 0,  0, 68, 0, 68, 0, 251, 0, 4095,
                        0, 68, 0,  0, 68, 0, 68, 0, 247, 0, 4095,
                        0, 68, 68, 68,68, 0, 68, 0, 239, 0, 4095,
                        0, 68, 0,  0, 68, 0, 68, 0, 223, 0, 4095,
                        0, 68, 0,  0, 68, 0, 68, 0, 191, 0, 4095,
                        0, 0,  0,  0, 0,  0, 0,  0, 127, 0, 4095])
                        

try:
    gather_dma, buffer_dma, reset_write_dma, reset_read_dma, sm0 = transfer_data(input)
    tim = Timer(mode=Timer.PERIODIC, period=1000, callback=update_image_flag_set)
    toggle = False
    while True:
        if UPDATE_IMAGE_FLAG:
            UPDATE_IMAGE_FLAG = 0
            if toggle == False:
                for i in range(0, len(input)):
                    input[i] = input_red[i]
                toggle = True
            else:
                for i in range(0, len(input)):
                    input[i] = input_blue[i]
                toggle = False
            print(addressof(input))       
            print("Testing DMAs. Toggle = ", toggle)
        
except KeyboardInterrupt:
    print("Turning off DMAs and StateMachine...")
    gather_dma.active(0)
    gather_dma.close()
    buffer_dma.active(0)
    buffer_dma.close()
    reset_write_dma.active(0)
    reset_write_dma.close()
    reset_read_dma.active(0)
    reset_read_dma.close()
    
    sm0.active(0)
    tim.deinit()
    print("DMAs and StateMachine turned off.")
    
    





