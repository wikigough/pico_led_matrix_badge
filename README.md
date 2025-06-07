# pico_led_matrix_badge

Using a Raspberry Pi Pico W (RP2040) and a Pimoroni 16x7 LED Matrix board using DMAs and State Machine(s), so that the entire animation update process 
can be run in the background (non-blocking).
The reason for this development is because this DMA chain is intended to be run with the WiFi module working and used on the Pico W, where the use of it
causes significant matrix flicker if background processes arent used.

## Overview
This repo has the following files:
- dma_with_pio_no_interrupt.py:
    - tested script which uses 4 DMAs to update a fixed image continuously, and triggers a change in the displayed image every second using the main CPU. 
    - the script therefore runs the matrix image update in the background, but is not able to run an animation in the background (i.e. updating where the 
      gather DMA reads from).
- dma_animation.py:
     - a demonstration of using a PIO module as a 1Hz trigger so that the animation update can be run in the background.
     - NOTE: use of two PIO modules (the state machine that updates the matrix and the 1Hz trigger), caused issues with the Pico W, as I think the WiFi module
       uses up remaining available state machines.
     - This doesnt use the matrix PIO, only demonstrates how we can implement frame updates. A combination of dma_animation.py
       and dma_with_pio_no_interrupt.py would be required.
 - pio.py:
     - State Machine definition for talking to the Pimoroni 16x7 LED Matrix.
 
 Files within the experiments folder may not be working and were used for interim tests. These will be deleted in the future, I'm just not good at using git.
 
 ## Detailed Explanation
 ### DMA and PIO modules
 For a detailed explanation on how the DMA and PIO modules work, please see the RP2040 datasheet.
 In terms of how the rp2 library works, please see the rp2 documentation for the best source of information. 
 Commenting has been employed within the python scripts for additional clarity.
 
 ### Diagrams
 Graphical diagrams have been provided within this repo to aid understanding of how the two files work. 
 
 ## Usage
 ### Running
 This script was only tested using Thonny. The files were manually uploaded onto the RPico W using the Thonny IDE, then the scripts were run from the Thonny 
 IDE by clicking on the green play button in the IDE.
 
 Once running, the LED matrix should show a "HI" in alternating red and blue every second.
 
 ### Exiting scripts
 The dmas and state machines remain on in the background unless the DMAs and State Machines are safely exited.
 This means that any errors that occur within the main script may halt the main program, but the dmas and state machines will continue running. 
 It is recommended to use the following methods to exit the script:
  - Use Ctrl+C in the Thonny shell
  - Press the STOP button in Thonny.

 
 ## KNOWN ISSUES - dma_with_pio_no_interrupt.py
 1. Upon exiting script, either by using Ctrl+C or by pressing the "stop" button in Thonny, and then attempting to run again in Thonny, the script doesnt 
 run on the first restart, only on the second. This means that to restart the program, you need to do STOP -> START (doesnt run) -> STOP -> START (runs).
 
 
 ## KNOWN ISSUES - dma_animation.py
 None yet...
