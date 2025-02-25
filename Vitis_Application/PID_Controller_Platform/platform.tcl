# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\Vitis_Application\PID_Controller_Platform\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\Vitis_Application\PID_Controller_Platform\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {PID_Controller_Platform}\
-hw {C:\ECE544-EmbeddedSystemsFPGA\ECE544-ACTUAL-Project2\hdl\nexysa7fpga.xsa}\
-proc {microblaze_0} -os {freertos10_xilinx} -out {C:/ECE544-EmbeddedSystemsFPGA/ECE544-ACTUAL-Project2/Vitis_Application}

platform write
platform generate -domains 
platform active {PID_Controller_Platform}
bsp reload
bsp config stdin "mdm_1"
bsp config enable_stm_event_trace "false"
bsp config stdout "mdm_1"
bsp write
bsp reload
catch {bsp regenerate}
platform generate
platform active {PID_Controller_Platform}
platform config -updatehw {C:/ECE544-EmbeddedSystemsFPGA/ECE544-ACTUAL-Project2/hdl/nexysa7fpga.xsa}
platform generate -domains 
