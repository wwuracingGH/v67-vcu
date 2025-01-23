# Programming the VCU #
Step 0. Plug in the vcu (12v power and STlink)

Step 1. Clone the github repo (https://github.com/wwuracingGH/v66-vcu.git)

Step 2. Install arm eabi compiler (easier if you use Linux):
If on windows, you'll need both the [ARM GNU toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) and [MinGW64](https://www.mingw-w64.org/downloads/)  
WSL/Linux just run ‘sudo apt-get install arm-none-eabi-gcc’  
In either case, verify by running `arm-none-eabi-gcc --version` 

Step 3. Build and Install OpenOCD (go here https://github.com/xpack-dev-tools/openocd-xpack/releases and then scroll down to a version with windows binaries  and click on them), and add these to your [path environment variables](https://www.wikihow.com/Change-the-PATH-Environment-Variable-on-Windows) 
![image](https://github.com/user-attachments/assets/1e998733-c6b4-41d7-8f8b-7722a3cc2d15)  
Check that you can run it with `openocd -v`. You might have to log out and back in for your PATH changes to go into effect

Step 4. Make sure the drivers for the st-link are updated. Go into device manager, and make sure it’s recognized and not a generic usb device. If it isn’t, the drivers are on stms website: (https://www.st.com/en/development-tools/stsw-link009.html)

![image](https://github.com/user-attachments/assets/82b28174-9616-44c7-9f0e-a54d8152df83)

Step 5. Verify that your environment contains make by running `make --version`. If on windows, you should already have installed [MinGW64](https://www.mingw-w64.org/downloads/) in step 2. If you are using linux it should be standard.

Step 6. In the environment with both make and the EABI compiler, run “make”

Step 7. In an environment with openOCD and make, run “make program”. 

#### Special cases: ####
For step 7: If you don’t want to install make onto a windows env, copy the line under “program:” and replace $(Binary) with v66vcu.elf 

# VCU debugging with GDB #
Coming soon?

# Troubleshooting: #
VCU not connecting to bus.
1.	Wait

2.	Try hot unplugging and replugging the vcu while everything else is on the bus

3.	Pray to vcu gods

Car not turning on

1.	Check APPS calibration message from vcu, if both APPS pedals aren't at zero or if there's an APPS error the car won't go into RTD.

2.	panic 

# The APPS calc Function #
The majority of the logic for the car lies inside the APPS_calc() function.

### The APPS transfer function: ###
The first thing that happens to the raw adc values after being smoothed is that they are transformed into float space, from 0 (minimum apps travel) to 1 (maximum apps travel). The transformation can be thought of as something like this:

![image](https://github.com/user-attachments/assets/82b8ce61-63c3-4800-ba73-8265a70dbd88)

This is done through these lines of code:

![image](https://github.com/user-attachments/assets/164848af-453f-4a6e-a600-c7fd45fa4bc9)

 

### Faults: ###
There are three different fault states that can arise after the transformation has happened.
The first one is fault state 1. This occurs when either of the apps values goes outside a dead zone set by the SENSOR_MIN and SENSOR_MAX variables at the top of the file.
In this diagram the red represents values that will cause fault 1 to trip:

![image](https://github.com/user-attachments/assets/685be587-2c5f-46b4-ba5a-eaaf96ce7f39)

 
The code for this is as follows. Note that if the value is not between 0-1 but is between Sensor max and Sensor min, it is clamped to 0-1.  

![image](https://github.com/user-attachments/assets/a4c3f8b2-e860-4c66-8e94-cd74af04bab5)

Fault 2 is triggered when the sensors disagree with each other more than 10%

![image](https://github.com/user-attachments/assets/9e59000d-1289-4769-a17f-ce2130a92025)



Fault 3 is apps/bspd plausibility. If it is triggered through this line of code:

![image](https://github.com/user-attachments/assets/630bf02b-754b-47e4-baef-412d68d04d11)
 
It will persist until reset by this line 

![image](https://github.com/user-attachments/assets/732ba968-128d-43c1-9be1-797e19320f54)

### Torque request ###
If none of the faults are triggered, torque request is set by the opposite operation of apps transfer.

![image](https://github.com/user-attachments/assets/bb0affc3-dca9-44ff-a283-dfffce41775e)

  
# Calibrating the apps pedal in software: #
Step 1: make sure that neither of the apps pedal pots show a voltage of .5v or 4.5v in either maximum position, this means lost resolution. Make sure they are also separated by at least 20% (different by ~0.5 v), but preferably more (~1V is ideal for the pedal travel).

Step 1.5: if either of the last two aren’t fulfilled, take off the springs and adjust the set screws so that they are. You can also adjust the travel with the front and back screws but that’s probably more a dynamics thing ig.

Step 2: connect the VCU to the can bus – verify it’s working.

Step 3: look at the raw APPS values. Verify that there isn’t a deadzone (when you move the pedal, the values should always change, and not be stuck at 330 or 4092 or something)

Step 4: set the raw apps values in code. The minimum should be the lowest value that you see + 20 and the maximum should be the highest value you see – 20, as this gives a reasonable deadzone.
 
Step 5. Turn on ready to drive and look at the apps calibration can message. verify that there are no faults and both apps values are 0 at 0 pedal travel and 1000 at max. THIS IS IMPORTANT. THE CAR WILL NOT START IF YOU DON¬’T VERIFY THIS!!!!!! BOTH APPS NEED TO BE AT OR UNDER 0% FOR RTD
 
# Traction control: the algorithm #

Based off of this paper: [TBA]

# LLLLRTTSOS #

Why?

The design goals of LLLLRTTSOS (Low Level Linked List Real Time Task/event Scheduled Operating System):

1.	Longest LV acronym, making me the king of nonsense abreviations (obviously most important)

2.	Modularity and scalability, opening the door a crack for unit testing (which is just a fancy word for testing little bits and pieces of code at a time, most commonly a function or factory)

3.	Cut down on the crazy number of side effects (parts of a function that modify state outside the parameters or return value of a function). The old code had ALL side effects, which made it a bit of a nightmare to debug sometimes.

4.	Real time processing. The ability to time when messages will appear on the bus exactly will lead to best results for a lower bit rate bus. This was the big reason for switching to RTOS, which started the side effects roulette, which as it would be a nightmare to debug I ended up deciding to refactor the code before test days started.

5.	No heap. Malloc? More like segmentation fault! (6kb of ram 😔). This means everything is stack allocated, which means it is explicit and must always contain data, which limits some of the more object-oriented code.

6.	Some blend of NASA embedded stylings and generally better code quality. I think I’ve failed the most on this one lol, but I’m saving it for a finalizing refactor before comp. It’s a great read for when u have time for it if either of y’all want to check it out: https://ntrs.nasa.gov/api/citations/19950022400/downloads/19950022400.pdf. Good candidate for the wiring harness bible of the programming sub-subsystem, along with https://leanpub.com/patternsinc 

7.	More modularity leads to a more collaborative codebase. If we have a larger programming team next year (which I hope we will), we will be able to have more than one person working on the code.

8.	More encapsulation; less exposed state: at the beginning of the programming process, I slapped all of the car’s state into one giant struct at the top called “car_state”. This is bad programming practice.

Before:

![image](https://github.com/user-attachments/assets/0086875a-03a1-4f71-872f-410274a0d21b)

After:

![image](https://github.com/user-attachments/assets/ce8f3658-f4ed-4a96-b38d-5d2c0c0e2a8e)

Not having to add a separate timer for each function greatly speeds up the process for adding a new “task” as they’re called (will get to this in the next part) and speeds up the process of adding one to just a single line, instead of changing half a dozen things that you need to keep track of or the code will break.
 
 
So just what is this LLLLRTTSOS, anyways? Does it pair well with toast?

![image](https://github.com/user-attachments/assets/256c27c5-0e54-4ed0-9d54-f2b7dcd93204)

LLLLRTTSOS is the combination of a struct, called the kernel, that stores states, tasks, and events, and 7 helper functions that makes sure you don’t have to edit the kernel whenever you want to change something.
	
A **STATE** is the overarching behavior of the processor, that has both an entry function, called when the OS switches to that state, an exit function called when the OS switches to a different state, and a list of tasks that get executed while the state is selected.

A **TASK** is a recurring function. These are queued in the real time portion of the OS when the Update function occurs and execute in the non-rt portion. They don’t really “belong” to states, but states do call them.
	
An **EVENT** is a function that will call itself once sometime in the future and then destroy itself. A good example of this is the buzzer turning off, which is only called once. This ensures that you aren’t wasting valuable clock cycles bothering with nothing, and memory on something that happened once but will never happen in the future.

The kernel struct has wrappers that accommodate the creation and destruction of these tasks, events, and states, although the design guidelines mean it is unwise to edit the OS state directly (read: causes undefined behavior, could softlock the processor if you’re not careful). There is also 1 static instance of it, the rtos_scheduler, that functions all call to.

### API ###

In order to schedule tasks or events, there are 6 helper functions you really need to know about:

**RTOS_addState(void (\*entry)(), void (\*exit)()):** creates a new state with the entry and exit functions and no tasks, returning the index of that state.

**RTOS_switchState(uint8_t state):** Switches to the state at the index provided, calling the last states exit function and the current states entry function. It also by extension changes the tasks that are running.

**RTOS_scheduleEvent(void (\*function), int countdown):** Schedules FUNCTION to execute COUNTDOWN milliseconds in the future. 

**RTOS_scheduleTask(uint8_t state, void (\*function), int period):** Schedules FUNCTION within STATE to repeat every PERIOD milliseconds. 

**RTOS_Update(void):** Called inside systick() to update the rtos flags and also call events

**RTOS_ExecuteTasks(void):** Called inside the non-realtime portions of the program to execute any flagged tasks.
