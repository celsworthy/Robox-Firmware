General

Functionality specific to 3d printing lives in the src directory.  Library routines and include files specific to the Atmel microcontroller live in the at91lib and external_libs directories.  These library routines originate from Atmel / ARM example projects, and in most cases they are unmodified.

The Robox firmware does not use an operating system.  Nearly everything is done in the foreground, but there are a few interrupts (in order of descending priority):

1] to generate motor step and direction signals
2] to handle the USB hardware
3] to generate PWM for the heaters, head fan and ambient LED
4] 1ms timer tick (this doesn't do much and could easily be dispensed with)

Robox firmware is organised as a number of "objects".  Each object has a static struct variable to store its state.  This variable is considered private and is never accessed from outside the object.  Typically, an object has an "initialise_" call, which is invoked once just after reset, and a "maintain_" call which is invoked regularly thereafter.

The maintain_ calls always return having done whatever processing is immediately necessary; they never wait on anything.  This is generally described as "co-operative multi-tasking" - that is, multi-tasking is achieved without need for the intervention of an operating system.  There are, of course, parts of the firmware which do wait for something.  Whilst waiting, they call do_everything_else() repetitively.  do_everything_else() simply invokes all the maintain_ calls in turn, so that the co-operative multi tasking remains alive.

The one exception to the not-waiting rule is maintain_job(), in gcode.c.  This can execute GCode commands which require a wait, so it is not included in the list of calls invoked by do_everything_else().

The firmware supports both rev1 and rev2 main boards, which have significant differences.  On rev1 hardware, the X/Y/Z/Z motor drivers are on the main board, and each extruder contains a motor driver.  On rev2 hardware, there are 6 plug-in "stepstick" drivers on the main board, and the extruder contains very little electronics.  detect_hw_rev() in motion_hal.c detects the hardware rev.


Descriptions of objects

command.c

Deals with receipt of commands, and issuing of reports via USB.  There are callbacks which are invoked from USB library routines, in the context of USB interrupts.  These simply register that a USB transfer has completed; actual processing of incoming commands is done in the foreground by maintain_commands().  In normal operation with Automaker, maintain_commands() passes the received data to robox_command_handler().  However, there is also a backwards-compatible mode of operation which allows Robox to be used with Pronterface - in this case, the received data is assumed to be textual GCode and is passed straight to gcode_handler().  maintain_commands() also deals with USB disconnect / reconnect, by polling the state of USB vbus, and the state of the 24V supply.

The handling of the Robox execute_gcode command deserves some explanation: if the GCode command is a "reporting" command, that is a command which has no effect other than to cause something to be reported, it is executed immediately via gcode_handler(), and the response is included in the gcode_response report.  On the other hand, if it is a command which affects the state of the machine, or which cannot be guaranteed to execute immediately, it is queued via execute_gcode(), and an empty gcode_response report is issued.  All GCode commands fall into one or the other of these categories.

Note report_debug_info(), which is invoked in response to a report_debug_info command.  This is an "unofficial" Robox command for debug purposes, the intention being that report_debug_info() can be quickly modified to return whatever data might be required to debug a particular problem.  In this special case, we make an exception to the normal rule of data being private to objects.


eeprom.c

Deals with reading and writing of the head and reel EEPROMS, and writing of the head shift register.  The head EEPROM and shift register are linked, because they use the same scl and sda signals.  We can't update the head shift register during an I2C transaction with the head EEPROM, so it is necessary to use short single byte EEPROM accesses to avoid deferring updates of the head shift register excessively.  The reel EEPROM does not suffer from this restriction, but it gets treated the same way as the head EEPROM, as the same routine is used to service both of them.  To allow for the possibility that set_head_output() may be invoked from an interrupt context whilst an I2C transaction with the head EEPROM is in progress, a lock flag is used to defer the update of the head shift register until the I2C transaction is finished.

The EEPROM is divided into two "banks", each of which has a 4 byte CRC.  In the normal steady state, the two banks are identical.  When the data is changed, we update one bank, and once that has been verified, we update the other.  This makes the system tolerant of unexpected disconnection of the EEPROM, or power-down, as the EEPROM is never in a state where both banks are not valid.


file_system.c

Deals with the SD card, providing a simple circular file system, that is a system where the oldest files are automatically discarded to make space for new files.  The directory is stored in block 0 of the SD card (each block being 512 bytes).  All the other blocks are available for file storage.  The directory is limited to 15 files, excluding the file currently being written.

A special feature of the file system is that a file may be opened for reading whilst it is still open for writing.  This allows execution of a job to begin whilst it is still being transferred from Automaker.

The filing system tolerates removal / changing of the SD card during operation, though obviously if this is done whilst a file is open, it results in an error being reported.  The card detect switch is polled so as to detect the removal of the SD card, otherwise there would be a danger that the card could be changed without the firmware noticing, with the result that the copy of the directory in RAM would no longer agree with the directory on the SD card.


gcode.c

Deals with parsing of lines of GCode, and the execution, pausing, resuming and aborting of jobs.  A job is executed by opening an SD card file for read, and reading it into the job buffer a block at a time.  A separate execute buffer stores GCode commands sent by Automaker.  The execute buffer always gets priority, so it is possible (if unwise) for Automaker to insert commands whilst a job is running.  The normal use of this feature is to allow the user to jog the machine whilst the job is paused, for example to facilitate manual cleaning of the nozzle.


heaters.c

Deals with temperature control, and PWM generation (as this is mainly heater / fan related).  Mains voltage detection is done here, partly because the mains bed heater is the reason we need to know the mains voltage, and partly because the timer interrupt routine used for PWM generation lends itself well to regular and frequent sampling of the 230V detect signal.  Control of the ambient LED is also done here - it's not related to the heaters, but it is the one other thing that requires PWM.

The PWM frequency doesn't need to be high, but it's important that it's not too close to the mains frequency, otherwise undesirable beating effects could occur on the mains bed heater, which is driven by a triac which can only switch on mains zero-crossings.  For this reason, I chose 40Hz for the heater & fan PWM.  The ambient LED PWM works at double this frequency to avoid visible flickering.

It's a rather inefficient implementation, as the interrupt frequency is 256x the 40Hz PWM frequency - it would be possible to generate n channels of PWM with only n+1 interrupts per cycle, which would greatly reduce the mean processing overhead.


motion.c

Deals with the generation of step and direction signals for all seven step motors, including motion command queueing and ramping with look-ahead planning.  The step and direction signals are generated in a timer interrupt routine, the period of the hardware timer being varied to achieve acceleration and deceleration ramping.  When the motion queue is empty, the interrupts continue at a low idling rate so that the interrupt routine can keep checking the queue to spot when it is no longer empty.

add_block_to_buffer() adds a new move to the queue, and performs the look-ahead planning.  Each move in the queue has an associated end speed, that is the maximum speed which is allowable when the move completes.  Obviously the last move in the queue must have an end speed of zero, but the other moves may have significantly higher end speeds, depending on the distance to the end of the queue, and how much the direction changes from one move to the next.  When a new move is queued, the end speeds of all the moves in the queue are re-calculated.

The "length" of each move, that is the number of interrupts required to complete it, is determined by the number of microsteps moved by the axis which travels furthest.  For each axis, there is an "inc" value: this is an integer for efficiency, but in effect it's a value between 0 and 1.  The axis which moves furthest has an inc value of 1, and gets a microstep for each interrupt.  The other axes have lower inc values, which determine how sparsely they are microstepped.  For example, an inc value of 0.1 means the axis gets microstepped 1 out of 10 interrupts.  An inc value of 0.9 means the axis gets microstepped 9 out of 10 interrupts.

Because each axis must be moved an integer number of microsteps, there is potential for error accumulation, particularly when a large number of short moves occur.  add_block_to_buffer() deals with this by returning the actual distance moved, allowing the caller to carry the residues over to the next move.

For simplicity, the same ramp rate (in microsteps / sec^2) is applied to all axes.  The rationale is that the X and Y axes, which are most critical to performance, also have the most inertia.  Therefore, if the ramp rate is optimal for these axes, it will be somewhat conservative for the other axes, but this has very little impact on the overall speed of printing.  Axis speed limits, on the other hand, are specified individually.  There is also an overall speed limit, empirically chosen to prevent the next timer interrupt from occurring before the interrupt routine has finished dealing with the current one.  Note that if compiler optimisation is turned off for debug purposes, a much lower overall speed limit is required as the unoptimised ISR takes significantly longer to execute.

Homing operations are a special case, in that the distance to be moved is not known in advance.  The interrupt routine deals with homing by not decrementing the remaining length counter until the relevant homing switch has changed state.  The length therefore defines the amount of overtravel allowed by the mechanical arrangement of the switch.  This works out rather neatly, with the homing speed being as fast as it can be whilst permitting deceleration to a halt within the available overtravel.


motion_hal.c / motion_hal_r1.c / motion_hal_r2.c

A hardware abstraction layer to deal with the motor driver hardware, which is completely different on rev1 and rev2 boards.


position.c

This is like a layer above motion.c, dealing with absolute position, tool (nozzle) changes and tool offsets.  Processing of G0/G1 command lines is done here, as are the higher level aspects of homing, gantry levelling and bed levelling.

In the case of the extruder (E/D) axes, the absolute position corresponds to the remaining filament length.  Consequently, the position is initialised to the remaining length value from the reel EEPROM, and subsequently the EEPROM is kept updated as the position changes.  Note that the E/D axes are also special in that the position is in units of mm, but the moves specified in G0/G1 commands are in (volumetric) units of mm^3.  The conversion is done according to the filament diameter, which is obtained from the reel EEPROM.


Descriptions of other source files, which don't constitute objects

board_test.c

Used in production test of rev1 boards.  Does nothing in the product.


main.c

This is the top level of the firmware.  main() simply invokes the initialise_ functions for all the objects, then loops forever calling do_everything_else() and maintain_job().


parameters.c

Deals with parameters which can be adjusted by M commands, but are stored in flash so as to be persistent.


reprogram.c

Deals with updating the firmware from a file on the SD card.  To minimise the danger of "bricking" the Robox, reprogramming is only done if the file passes several tests, most important of which is a checksum.  After reprogramming, the microcontroller is reset, because this is the only way of proceeding - it's not possible to return to the calling context, as it no longer exists.

The reprogramming routine (and any subroutines it calls) must reside in RAM, as it's not possible to execute from flash whilst the flash is being reprogrammed.  The keyword __attribute__((section(".ramfunc"))) instructs the compiler to deal with this, though note that the firmware also plays a part in the process by doing a copy from flash to RAM at start-up; see ResetException().

Once the firmware file has been opened for read, the reprogramming routine avoids any further file system calls (as otherwise the entire file system object would need to reside in RAM).  This is possible because the firmware file is always contained in contiguous SD card blocks, so that it is trivial to read the data from the card interface hardware 4 bytes at a time, once the hardware has been set up.


samadc.c

This just deals with the 12 bit ADC, used to measure the various thermistors.


util.c

This contains a few general-purpose calls which are used throughout the firmware.


Building a firmware release

The firmware is compiled using Sourcery CodeBench Lite 2012.03-56.  The script build/build.bat invokes the compiler and performs the necessary file conversions.

The compilation process generates robox.elf, a file format which includes debug information for use by emulators etc.  A conversion utility is used to generate raw binary file robox.bin from robox.elf.  Another utility is used to generate robox.info from robox.elf; robox.info is a human-readable list of symbol values which is occasionally useful for debug purposes.  

Finally, the program anticheckbin is invoked to insert an antichecksum in robox.bin, so that the 32-bit checksum is zero (see tools/anticheckbin.c).  The firmware will only update from a file with zero checksum, as a precaution against accidental "bricking".
