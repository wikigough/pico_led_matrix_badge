#   Sending LED Matrix data over DMA chain.
#   Uses example from the documentation as a basis for DMA chaining:
#       https://docs.micropython.org/en/latest/library/rp2.DMA.html
#   Uses the pio from here (changed to pythonic version and slightly modified):
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


def transfer_data(input_list, buf, test=False):
    """
        transfer_data transfers data to either the output buf OR a pio defined within transfer_data,
        depending on the test value:
         - test == True: transfer_data transfers to buf and displays the image using StateMachine.put().
            this test is to separate the two functionalities - to show that the DMA chain is working,
            and to show that the PIO is working separately as intended. *working*
         - test == False: transfer_data transfers to the PIO TX FIFO register. *not working*
        
        Both cases use 2 DMAs chained to each other, each triggering each other. gather_dma chains to
        buffer_dma and buffer_dma chains back to gather_dma. Note: confusingly, using rp2.DMA()
        semi-randomly assigns the channel number.
        
        gather_dma, as per the example in the micropython documentation, directly populates the buffer_dma
        registers with the addresses of the start of each input_data row and the length of that row.
        
        Then, buffer_dma is triggered/turned to active by gather_dma once gather_dma writes to buffer_dma
        registers, and buffer_dma either dumps that data in the output buffer (if test==True) or writes
        data to the PIO TX FIFO.
        
        In the case of test==False, the state machine has an irq(rel(0)) that will reconfigure gether_dma to
        restart from the original address. This has not been tested yet.

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
    # Pack up length/address pairs to be sent to the registers.
    gather_list = array("L")

    for row in input_list:
        gather_list.append(len(row))
        gather_list.append(addressof(row))

    # Unsure why we place zeroes at the end, but we do.  
    gather_list.append(0)
    gather_list.append(0)
    print(gather_list)  # --> this is just to check that the gather_list has been indeed populated
    print("Gather list address start = ", addressof(gather_list)) # --> this is to check against the DMA registers
    print("Buffer address start = ", addressof(buf)) # --> this is to check against the DMA registers
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- GATHER DMA -------------------------------------------#
    # When writing to the registers of the second DMA channel, we need to wrap the
    # write address on an 8-byte (1<<3 bytes) boundary. We write to the ``TRANS_COUNT``
    # and ``READ_ADD_TRIG`` registers in the last register alias (registers 14 and 15).
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
    
    def trigger_gather_dma(sm0):
        """
            used by State Machine 0 when the full input_data transfer has been completed.
            NOT TESTED

        """
        print("gather_dma trig")
        gather_dma.config(
            read=addressof(gather_addr), 
            write=buffer_dma.registers[14:16],
            count=2, 
            ctrl=gather_ctrl, 
            trigger=True
        )
    
    
    gather_dma.config(
        read=addressof(gather_list), write=buffer_dma.registers[14:16],
        count=2, ctrl=gather_ctrl, trigger=False
    )
    #----------------------------------------------------------------------------------------------------#
    
    #--------------------------------------------- BUFFER DMA -------------------------------------------#
    if test:
        # When copying the data, the transfer size is single bytes, and when completed we need
        # to chain back to the start another gather DMA transaction.
        buffer_ctrl = buffer_dma.pack_ctrl(
            enable=True,
            high_pri=False,
            size=2,
            inc_read=True,   # original = True
            inc_write=True, # original = True
            ring_size=0,
            ring_sel=False,
            treq_sel=63,      # original = 63
            irq_quiet=True,   # original = True
            bswap=False,
            sniff_en=False,
            chain_to=gather_dma.channel
        )
        
        
        # The read and count values will be set by the other DMA channel.
        buffer_dma.config(write=addressof(buf), ctrl=buffer_ctrl, trigger=False)
    
    else:
        # When copying the data, the transfer size is single bytes, and when completed we need
        # to chain back to the start another gather DMA transaction.
        buffer_ctrl = buffer_dma.pack_ctrl(
            enable=True,
            high_pri=False,
            size=2,
            inc_read=False,   # original = True
            inc_write=False,  # original = True
            ring_size=0,
            ring_sel=False,
            treq_sel=4,       # original = 63
            irq_quiet=True,
            bswap=False,
            sniff_en=False,
            chain_to=buffer_dma.channel
        )
        
        
        # The read and count values will be set by the other DMA channel.
        buffer_dma.config(write=TXF0_addr, ctrl=buffer_ctrl, trigger=False)
        
    
        
        # state machine 0 irq - not tested.
        sm0.irq(handler=trigger_gather_dma, trigger=1, hard=True)
        
    #----------------------------------------------------------------------------------------------------#
        
    #--------------------------------------------- ACTIVATION -------------------------------------------#    
    # display initial settings before we kick off the data transfer process
    
    print("Gather DMA channel: ", gather_dma.channel, ", Buffer DMA channel: ", buffer_dma.channel)
    
    print("Gather DMA registers: ")
    for register in gather_dma.registers:
        print(register)
    
    print("Buffer DMA registers: ")
    for register in buffer_dma.registers:
        print(register)
    
    # activate state machine
    sm0.active(1)
    
    # Set the transfer in motion.
    gather_dma.active(1)
    #buffer_dma.active(1) - dont actually need to set this one off, as gather_dma will trigger the buffer_dma.

    # Wait until all the register values have been sent
    end_address = addressof(gather_list) + len(gather_list)
    while gather_dma.read <= end_address:
        print("Gather DMA read: ", gather_dma.read)
        print("Buffer DMA read: ", buffer_dma.read, ", buffer DMA write: ", buffer_dma.write, ", buffer DMA count: ",buffer_dma.count )
        print(gather_dma.active(), buffer_dma.active())
        time.sleep(2)
       
    
    print("Done.") # this is only used for the test.
    #print(gather_dma.active())
    #print(buffer_dma.active())
    gather_dma.close()
    buffer_dma.close()
    
    # returning sm0 for further testing.
    return sm0
    #----------------------------------------------------------------------------------------------------#


#### MAIN ####

input = [
    array("L", [0, 0,  0,  0, 0,  0, 0,  0, 253, 0, 4095]),
    array("L", [0, 34, 0,  0, 34, 0, 34, 0, 251, 0, 4095]),
    array("L", [0, 34, 0,  0, 34, 0, 34, 0, 247, 0, 4095]),
    array("L", [0, 34, 34, 34,34, 0, 34, 0, 239, 0, 4095]),
    array("L", [0, 34, 0,  0, 34, 0, 34, 0, 223, 0, 4095]),
    array("L", [0, 34, 0,  0, 34, 0, 34, 0, 191, 0, 4095]),
    array("L", [0, 0,  0,  0, 0,  0, 0,  0, 127, 0, 4095])
]

output = array('L', [])
for i in range(0, 77):
    output.append(0)


print(output)
sm0 = transfer_data(input, output, test=False)
print(output)

while True:
    for item in output:
        sm0.put(item)




