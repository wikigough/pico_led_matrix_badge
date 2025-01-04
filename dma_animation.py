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

TXF0_addr = const(0x50200010)   # address of TX FIFO register for State Machine 0
RXF0_addr = const(0x50200020)   # address of RX FIFO register for State Machine 0

TXF1_addr = const(0x50200014)   # address of TX FIFO register for State Machine 0
RXF1_addr = const(0x50200024)   # address of RX FIFO register for State Machine 0


def transfer_animation_data(input_array, buf):
    """
        transfer_data transfer the contents of the input_array to buf.
        
        The transfer is repeated, and occurs every second. The transfer rate is the fastest it can be
        (treq_sel=63), but the repeat rate is slow (1Hz in this example). Because the repeat rate is so
        slow, we need to use a PIO with an interrupt to obtain the desired repeat rate. If something much
        faster is desired (eg something >=2kHz repeat rate), it could be possible to re-write this by
        setting treq_sel to a divided timer (while also making sure that the timer division X and Y
        registers have been set correctly).
        
        input_array is a set of animation frames:
        input_array = [ array("L", [34, 34, ... ]), array("L", [45, 45, ... ]), ... ]
                      [ ^ frame 0                 , ^ frame 1                 , ... ]
        
        the output buf is then updated with frame 0, then a second later by frame 1 and so on until the end
        of the input_array is reached, and then the output buf is populated with frame 0 and the cycle repeats.
        
        
        
        
        

    """
      
    #--------------------------------------------- DMA INIT ---------------------------------------------#
    # We use two DMA channels. The first sends lengths and source addresses from the gather
    # list to the registers of the second. The second copies the data itself.
    gather_animation_dma = DMA()
    buffer_animation_dma = DMA()
    restart_animation_dma = DMA()
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER LIST ------------------------------------------#
    # Pack up length/address pair of input_array to be sent to the buffer_animation_dma registers.
    # This only works for a single input_array = array("L", [])
    gather_ani_list = array("L")
    
    for row in input_array:
        gather_ani_list.append(addressof(buf))
        gather_ani_list.append(len(row))       # CH0_AL3_TRANS_COUNT
        gather_ani_list.append(addressof(row)) # CH0_AL3_READ_ADDR_TRIG
        #gather_ani_list.append(0)
        
    
    gather_ani_list_addr_start = addressof(gather_ani_list) # for gather_animation_dma range check
    gather_ani_list_addr_end = gather_ani_list_addr_start + len(gather_ani_list)*4
    
    print(gather_ani_list_addr_start, gather_ani_list_addr_end)
    print(gather_ani_list)
    
    #----------------------------------------------------------------------------------------------------#
    
  
    
    #--------------------------------------------- GATHER DMA -------------------------------------------#
    # When writing to the registers of the second DMA channel, we need to wrap the
    # write address on an 8-byte (1<<3 bytes) boundary. We write to the ``TRANS_COUNT``
    # and ``READ_ADD_TRIG`` registers in the last register alias (registers 14 and 15
    # respectively).
    gather_animation_ctrl = gather_animation_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=True,
        inc_write=True,
        ring_size=4,
        ring_sel=True,
        treq_sel=63,
        irq_quiet=False,
        bswap=False,
        sniff_en=False,
        chain_to=buffer_animation_dma.channel
    )   
    
    gather_animation_dma.config(
        read=addressof(gather_ani_list), write=buffer_animation_dma.registers[13:16],
        count=3, ctrl=gather_animation_ctrl, trigger=False
    )
    
    def gather_animation_dma_irq(gather_animation_dma):
        print("gather_animation_dma irq")
    
    gather_animation_dma.irq(handler=gather_animation_dma_irq, hard=False)
    
    #----------------------------------------------------------------------------------------------------#
    
       
        
    #--------------------------------------------- BUFFER DMA -------------------------------------------#
    
    # for writing to an output buffer only
    buffer_animation_ctrl = buffer_animation_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=True,   
        inc_write=True,             # inc_write required for writing to output buffer.
        ring_size=0,                
        ring_sel=False,
        treq_sel=63,      
        irq_quiet=False,             # no need for restarting gather_animation_dma using irq in the test, so this is set to quiet.
        bswap=False,
        sniff_en=False,
        chain_to=restart_animation_dma.channel # this could be chained to self and then trigger interrupt, but I want to save on interrupts.
    )
        
        
    # The read and count values will be set by the other DMA channel.
    buffer_animation_dma.config(write=addressof(buf), ctrl=buffer_animation_ctrl, trigger=False)
        
        
    def buffer_animation_dma_irq_handler(buffer_animation_dma):
        print("buffer_animation_dma irq")
            

    buffer_animation_dma.irq(handler=buffer_animation_dma_irq_handler, hard=False)
    #----------------------------------------------------------------------------------------------------#
 
    #--------------------------------------------- RESTART DMA -------------------------------------------#
    # for writing to an output buffer only
    restart_animation_ctrl = restart_animation_dma.pack_ctrl(
        enable=True,
        high_pri=False,
        size=2,
        inc_read=True,   
        inc_write=False,             # inc_write required for writing to output buffer.
        ring_size=0,
        ring_sel=False,
        treq_sel=1,                  # = DREQ_PIO0_TX0, 1 = DREQ_PIO0_TX1
        irq_quiet=False,             # no need for restarting gather_animation_dma using irq in the test, so this is set to quiet.
        bswap=False,
        sniff_en=False,
        chain_to=restart_animation_dma.channel 
    )
    
    def restart_animation_dma_irq(restart_animation_dma):
        print("restart_animation_dma irq")
        
    # The read and count values will be set by the other DMA channel.
    restart_animation_dma.config(write=TXF1_addr, ctrl=restart_animation_ctrl, count=1, trigger=False)
    
    restart_animation_dma.irq(handler=restart_animation_dma_irq, hard=False)
    #----------------------------------------------------------------------------------------------------#
    
    #-------------------------------------------- PIO CHOKE SETUP ------------------------------------------#
    # Used for pacing the gather_animation_dma.
    # The pacing timer produces TREQ assertions at a rate set by ((X/Y) * sys_clk). This equation is evaluated every
    # sys_clk cycles and therefore can only generate TREQs at a rate of 1 per sys_clk (i.e. permanent TREQ) or less.
    # From section 2.15.3, sys_clk, or clk_sys is 125MHz.
    # this means the maximum division we an get is 125MHz / (1/2^16) = 1.9kHz. This is not low enough,
    # so we need to use a PIO to choke the DMA even further.
    @rp2.asm_pio(fifo_join = rp2.PIO.JOIN_TX, autopull=True, pull_thresh=32, autopush=True, push_thresh=32)
    def trigger_dma_1hz():
        # we are limited by delay number and what we can set x to.
        # Cycles: 1 + 7 + 32 * (29 + 1 + 1) = 1000        
        # repeat twice with minimum freq=2000
        # then you get 1Hz.
        wrap_target()
        
        pull(ifempty)               # pull data. 
        out(null, 32)               # discard input 
        
        set(x, 31)                  [6]
        label("delay_1")
        nop()                       [29] # [29] means repeat 29 times.
        jmp(x_dec, "delay_1")
        
        set(x, 31)                  [6]
        label("delay_2")
        nop()                       [29] # [29] means repeat 29 times.
        jmp(x_dec, "delay_2")

        irq(rel(0))
        
        wrap()
        
    def sm_irq(sm):
        print("sm_irq. Buffer at sm_irq: ", buf)
        # if we've reached the end of the whole list of frames, restart from beginning.
        if gather_animation_dma.read >= gather_ani_list_addr_end:
            gather_animation_dma.config(read=addressof(gather_ani_list), write=buffer_animation_dma.registers[13:16], trigger=True )
        else:
            # note it is necessary to set write as the buffer_animation_dma registers again. 
            gather_animation_dma.config(write=buffer_animation_dma.registers[13:16], trigger=True)

    
    rp2.PIO(0).remove_program()
    sm = rp2.StateMachine(1, trigger_dma_1hz, freq=2000)
    sm.irq(handler=sm_irq, hard=False)
    
    
    #----------------------------------------------------------------------------------------------------#


    #--------------------------------------------- ACTIVATION -------------------------------------------#    
    
    # activate state machine
    sm.active(1)
    
    # activate gather_animation_dma. Do not activate buffer_animation_dma, as gather_animation_dma will trigger it once it properly sets
    # buffer_animation_dma up.
    gather_animation_dma.active(1)
    
    print("DMAs commenced.")
    
    return gather_animation_dma, buffer_animation_dma, restart_animation_dma, sm

    #----------------------------------------------------------------------------------------------------#


#### MAIN ####


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

animation = [input_red, input_blue]
                        

output = array('L', [])
for i in range(0, 77):
    output.append(0)


print("Output buffer before DMAs set in motion: ", output)
gather_animation_dma, buffer_animation_dma, restart_animation_dma, sm = transfer_animation_data(animation, output)

while True:
    pass
    





