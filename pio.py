"""
    This script is an initial PIO test run on the RPi Pico WH. 
    This is heavily inspired by the following sources:
     - spi example here: https://github.com/raspberrypi/pico-micropython-examples/blob/master/pio/pio_spi.py
     - pio example here: https://github.com/pimoroni/pimoroni-pico/blob/main/libraries/pico_unicorn/pico_unicorn.pio
     - youtube video explaining PIO here: https://www.youtube.com/watch?v=yYnQYF_Xa8g
     - and last but not least, the actual documentation: https://docs.micropython.org/en/latest/library/rp2.html#rp2.asm_pio

    Current version doesnt have the brightness control working properly. It's all coming out fairly dim. Needs a fix.
    
    Further info on LED drivers:
    PIO communicates with TLC59283.This has a resistor that sets the max current throught the LED, so brightness control has to be done by controlling the duty cycle of when the LEDs are on and when they are off. 
     
"""

import rp2

"""

    ; set pins:
    ; 0: data (base) - pin 11 - GP8
    ; 1: clock - pin 12 - GP9
    ; 2: latch - pin 14 - GP10
    ; 3: blank - pin 15 - GP11

    ; sideset pin: clock - pin 12

    ; out pins:
    ; 0: row 6 select - pin 21 - GP16
    ; 1: row 5 select - pin 22 - GP17
    ; 2: row 4 select - pin 24 - GP18
    ; 3: row 3 select - pin 25 - GP19
    ; 4: row 2 select - pin 26 - GP20
    ; 5: row 1 select - pin 27 - GP21
    ; 6: row 0 select - pin 29 - GP22


"""

#-------------------------------------
#   PIO DEFINE
#-------------------------------------#

"""
   PIO that updates the output of each ROW. So this is one ROW at a time, iterating through each column.
   Set base, out base, sideset base not defined here, they are defined when the PIO is initialised.
   
"""

@rp2.asm_pio(fifo_join = rp2.PIO.JOIN_TX, out_shiftdir=rp2.PIO.SHIFT_RIGHT, autopull=True, pull_thresh=8, sideset_init=rp2.PIO.OUT_HIGH, out_init=(rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW,rp2.PIO.OUT_LOW), set_init=(rp2.PIO.OUT_HIGH,rp2.PIO.OUT_HIGH,rp2.PIO.OUT_HIGH,rp2.PIO.OUT_HIGH))
def shift_reg():
    
    wrap_target()                         # wrapper for the loop.
    set(y, 15)                            # 15 because `jmp` test is pre decrement. Y is used for looping through the data               
    pull(ifempty)                         # pulls data to the OSR
    
    label("pixels")                       # start of bitloop / pixel output loop. 
    out(null, 1)                          # dummy bit used to align pixel data to nibbles - discard
    
    # RED BIT
    out(x, 1)                  .side(0)   # pull in first bit from OSR into register X, clear clock
    set(pins, 0)                          # clear data bit (maintain blank)
    jmp(not_x, "endr")                    # if bit was zero jump to endr
    set(pins, 1)                          # set data bit (maintain blank)
    
    label("endr")
    nop()                      .side(1)
    
    # GREEN BIT
    out(x, 1)                  .side(0)
    set(pins, 0)
    jmp(not_x, "endg")
    set(pins, 1)
    
    label("endg")
    nop()                      .side(1)
    
    # BLUE BIT
    out(x, 1)                  .side(0)
    set(pins, 0)
    jmp(not_x, "endb")
    set(pins, 1)
    
    label("endb")
    nop()                      .side(1)
    
    # Jump back to start of pixel loop, if y=0, then pull. 
    jmp(y_dec, "pixels")
    
    
    # select active row
    out(null, 1)                          # discard dummy bit
    out(pins, 7)                          # output row selection mask - this basically pulls in the 7 bits of the remaining mask byte and just dumping whatever it sees on that byte onto the OUT pins.
    
    set(pins, 4)                         # set latch pin to output data on shift registers (while keeping blank low)
    
    # now time to do brightness control. Note output is currently high on this row.
    out(x, 12)                            # now get brightness control ON count into x.
    
    label("bcd_count_on")  
    jmp(x_dec, "bcd_count_on")
    
    out(y, 12)                           # now get brightness control OFF count into y.
    set(pins, 8)                         # set blank high and latch low during this off period of dimming. (8 = 0b1000)
    
    label("bcd_count_off")
    jmp(y_dec, "bcd_count_off")

    irq(rel(0)) 
    
    wrap()


        

