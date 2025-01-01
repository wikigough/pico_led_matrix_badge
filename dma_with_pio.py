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


def transfer_data(input_array, buf, test=False):
    """
        transfer_data transfers data to either the output buf OR a pio defined within transfer_data,
        depending on the test value:
         - test == True: transfer_data transfers to buf and displays the image using StateMachine.put().
            this test is to separate the two functionalities - to show that the DMA chain is working,
            and to show that the PIO is working separately as intended.
         - test == False: transfer_data transfers to the PIO TX FIFO register. 
        
        Both cases use DMA chaining:
         - gather_dma chains to the buffer_dma,
         - buffer_dma *chains to itself*, meaning that chaining is disabled for this channel.
         
        Specifically in the case where test=False, buffer_dma generates an interrupt at end of transfer.
        Within the interrupt generated by the buffer_dma, the gather_dma is re-configured and retriggered.
        The reason for not chaining back the buffer_dma back to the gather_dma, is that we want a time during
        which gather_dma is not running while we reconfigure it. I've found that reconfiguring a dma while
        it was running led to memory errors (MemoryError: ). So a brief pause is necessary.
        
        gather_dma, as per the example in the micropython documentation (see link above), directly populates
        the buffer_dma registers with:
         - the number of transfers (COUNT register)
         - the address of the start of the input_array array.
        
        Then, buffer_dma is triggered/turned to active by gather_dma once gather_dma writes to buffer_dma
        registers, and buffer_dma either dumps that data in the output buffer (if test==True) or writes
        data to the PIO TX FIFO.
        

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
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER LIST ------------------------------------------#
    # Pack up length/address pair of input_array to be sent to the buffer_dma registers.
    # This only works for a single input_array = array("L", [])
    gather_list = array("L")
    
    gather_list.append(len(input_array))
    gather_list.append(addressof(input_array))
    
    gather_list_addr = addressof(gather_list)
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER DMA -------------------------------------------#
    # When writing to the registers of the second DMA channel, we need to wrap the
    # write address on an 8-byte (1<<3 bytes) boundary. We write to the ``TRANS_COUNT``
    # and ``READ_ADD_TRIG`` registers in the last register alias (registers 14 and 15
    # respectively).
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
    if test:
        # for writing to an output buffer only
        buffer_ctrl = buffer_dma.pack_ctrl(
            enable=True,
            high_pri=False,
            size=2,
            inc_read=True,   
            inc_write=True,             # inc_write required for writing to output buffer.
            ring_size=0,
            ring_sel=False,
            treq_sel=63,      
            irq_quiet=True,             # no need for restarting gather_dma using irq in the test, so this is set to quiet.
            bswap=False,
            sniff_en=False,
            chain_to=buffer_dma.channel 
        )
        
        
        # The read and count values will be set by the other DMA channel.
        buffer_dma.config(write=addressof(buf), ctrl=buffer_ctrl, trigger=False)
    
    else:
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
            irq_quiet=False,
            bswap=False,
            sniff_en=False,
            chain_to=buffer_dma.channel 
        )
        
        
        def buffer_dma_irq_handler(buffer_dma):
            """
                Interrupt routine after buffer_dma finishes its transfer.
                The IRQ re-configures gather_dma and triggers it to run again.
                Used only for continuous operation.
                
            """
            try:
                gather_dma.config(
                    read=addressof(gather_list), write=buffer_dma.registers[14:16],
                    count=2, ctrl=gather_ctrl, trigger=True
                )
            except ValueError as e:
                print("Value Error: ", e)
 
        # The read and count values will be set by the other DMA channel.
        buffer_dma.config(write=TXF0_addr, ctrl=buffer_ctrl, trigger=False)

        buffer_dma.irq(handler=buffer_dma_irq_handler, hard=False)
    #----------------------------------------------------------------------------------------------------#
        
    #--------------------------------------------- ACTIVATION -------------------------------------------#    
    
    # activate state machine
    sm0.active(1)
    
    # activate gather_dma. Do not activate buffer_dma, as gather_dma will trigger it once it properly sets
    # buffer_dma up.
    gather_dma.active(1)
    
    print("DMAs commenced.")
    
    return gather_dma, buffer_dma, sm0

    #----------------------------------------------------------------------------------------------------#


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
                        

output = array('L', [])
for i in range(0, 77):
    output.append(0)

test = False


try:
    if test:
        print("Output buffer before DMAs set in motion: ", output)
        gather_dma, buffer_dma, sm0 = transfer_data(input, output, test=test)
        print(output)
        
        while True:
            for item in output:
                sm0.put(item)
    else:
        gather_dma, buffer_dma, sm0 = transfer_data(input, output, test=test)
        toggle = False 
        while True:
            time.sleep(1)
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
    sm0.active(0)
    print("DMAs and StateMachine turned off.")
    
    




